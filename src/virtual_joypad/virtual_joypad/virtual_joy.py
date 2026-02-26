"""
Virtual Xbox Controller GUI for ROS2.

Provides a PyQt6-based GUI emulating an Xbox controller that publishes
sensor_msgs/Joy messages. Features sticky sticks, draggable triggers,
and toggleable buttons.
"""

import sys
import signal
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPainterPath, QLinearGradient

FREQ = 10


class VirtualXbox(Node, QWidget):
    # Color scheme
    COLORS = {
        'background': QColor(26, 26, 26),
        'controller_body': QColor(45, 45, 45),
        'controller_edge': QColor(60, 60, 60),
        'stick_area': QColor(35, 35, 35),
        'stick_knob': QColor(80, 80, 80),
        'stick_knob_active': QColor(76, 194, 255),
        'button_inactive': QColor(60, 60, 60),
        'button_active': QColor(76, 194, 255),
        'A': QColor(16, 124, 16),
        'B': QColor(232, 17, 35),
        'X': QColor(0, 120, 215),
        'Y': QColor(255, 185, 0),
        'trigger_bg': QColor(40, 40, 40),
        'trigger_fill': QColor(76, 194, 255),
        'text': QColor(200, 200, 200),
        'text_dark': QColor(30, 30, 30),
        'dpad': QColor(50, 50, 50),
        'dpad_active': QColor(76, 194, 255),
        'reset_button': QColor(70, 70, 70),
        'reset_button_hover': QColor(90, 90, 90),
    }

    def __init__(self):
        Node.__init__(self, "virtual_xbox")
        QWidget.__init__(self)

        self.joy_pub = self.create_publisher(Joy, "joy", 10)
        self.setWindowTitle("ROS2 Virtual Xbox Controller")
        self.setFixedSize(700, 500)
        self.setMouseTracking(True)

        # Axes: [LX, LY, LT, RX, RY, RT, DPadX, DPadY]
        self.axes = [0.0] * 8
        # Buttons: [A, B, X, Y, LB, RB, Back, Start, Xbox, L3, R3]
        self.buttons_state = [0] * 11

        # Stick positions (normalized -1 to 1)
        self.left_stick = QPointF(0, 0)
        self.right_stick = QPointF(0, 0)

        # Trigger values (0 to 1)
        self.left_trigger = 0.0
        self.right_trigger = 0.0

        # Drag state
        self.dragging_left_stick = False
        self.dragging_right_stick = False
        self.dragging_left_trigger = False
        self.dragging_right_trigger = False

        # Hover states for reset buttons
        self.hover_reset_left = False
        self.hover_reset_right = False

        # Define interactive regions (will be set in paintEvent based on size)
        self._update_regions()

        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_joy)
        self.timer.start(int(1000 // FREQ))

    def _update_regions(self):
        """Update all interactive regions based on widget size"""
        w, h = self.width(), self.height()

        # Stick areas (circles)
        self.left_stick_center = QPointF(150, 320)
        self.right_stick_center = QPointF(550, 320)
        self.stick_radius = 60
        self.knob_radius = 20

        # Trigger areas
        self.left_trigger_rect = QRectF(60, 80, 120, 25)
        self.right_trigger_rect = QRectF(520, 80, 120, 25)

        # Bumper areas
        self.left_bumper_rect = QRectF(60, 45, 100, 30)
        self.right_bumper_rect = QRectF(540, 45, 100, 30)

        # ABXY button positions (centers)
        abxy_center = QPointF(550, 180)
        self.button_radius = 22
        self.button_positions = {
            'A': QPointF(abxy_center.x(), abxy_center.y() + 45),
            'B': QPointF(abxy_center.x() + 45, abxy_center.y()),
            'X': QPointF(abxy_center.x() - 45, abxy_center.y()),
            'Y': QPointF(abxy_center.x(), abxy_center.y() - 45),
        }

        # D-Pad
        self.dpad_center = QPointF(150, 180)
        self.dpad_size = 35

        # Center buttons (Back, Xbox, Start)
        self.center_buttons = {
            'Back': QRectF(280, 180, 40, 25),
            'Xbox': QRectF(330, 170, 40, 40),
            'Start': QRectF(380, 180, 40, 25),
        }

        # L3, R3 buttons
        self.l3_rect = QRectF(130, 400, 40, 25)
        self.r3_rect = QRectF(530, 400, 40, 25)

        # Reset buttons
        self.reset_left_rect = QRectF(90, 400, 35, 25)
        self.reset_right_rect = QRectF(575, 400, 35, 25)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        painter.fillRect(self.rect(), self.COLORS['background'])

        # Draw controller body
        self._draw_controller_body(painter)

        # Draw triggers
        self._draw_triggers(painter)

        # Draw bumpers
        self._draw_bumpers(painter)

        # Draw D-Pad
        self._draw_dpad(painter)

        # Draw ABXY buttons
        self._draw_abxy(painter)

        # Draw center buttons
        self._draw_center_buttons(painter)

        # Draw sticks
        self._draw_stick(painter, self.left_stick_center, self.left_stick, "L", self.dragging_left_stick)
        self._draw_stick(painter, self.right_stick_center, self.right_stick, "R", self.dragging_right_stick)

        # Draw L3, R3 buttons
        self._draw_l3_r3(painter)

        # Draw reset buttons
        self._draw_reset_buttons(painter)

        # Draw title
        painter.setPen(self.COLORS['text'])
        painter.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        painter.drawText(QRectF(0, 10, self.width(), 30), Qt.AlignmentFlag.AlignCenter, "Virtual Xbox Controller")

    def _draw_controller_body(self, painter):
        """Draw the controller body shape"""
        path = QPainterPath()

        # Create a rounded controller shape
        body_rect = QRectF(40, 40, 620, 420)
        path.addRoundedRect(body_rect, 40, 40)

        # Gradient for 3D effect
        gradient = QLinearGradient(body_rect.topLeft(), body_rect.bottomRight())
        gradient.setColorAt(0, self.COLORS['controller_edge'])
        gradient.setColorAt(0.5, self.COLORS['controller_body'])
        gradient.setColorAt(1, QColor(30, 30, 30))

        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(self.COLORS['controller_edge'], 2))
        painter.drawPath(path)

    def _draw_triggers(self, painter):
        """Draw LT and RT triggers"""
        # Left trigger
        self._draw_single_trigger(painter, self.left_trigger_rect, self.left_trigger, "LT")
        # Right trigger
        self._draw_single_trigger(painter, self.right_trigger_rect, self.right_trigger, "RT")

    def _draw_single_trigger(self, painter, rect, value, label):
        """Draw a single trigger"""
        # Background
        painter.setBrush(self.COLORS['trigger_bg'])
        painter.setPen(QPen(self.COLORS['controller_edge'], 1))
        painter.drawRoundedRect(rect, 5, 5)

        # Fill based on value
        fill_width = rect.width() * value
        fill_rect = QRectF(rect.x(), rect.y(), fill_width, rect.height())
        painter.setBrush(self.COLORS['trigger_fill'])
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(fill_rect, 5, 5)

        # Label
        painter.setPen(self.COLORS['text'])
        painter.setFont(QFont("Arial", 10))
        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, f"{label}: {int(value * 100)}%")

    def _draw_bumpers(self, painter):
        """Draw LB and RB bumpers"""
        # LB (index 4)
        color = self.COLORS['button_active'] if self.buttons_state[4] else self.COLORS['button_inactive']
        painter.setBrush(color)
        painter.setPen(QPen(self.COLORS['controller_edge'], 1))
        painter.drawRoundedRect(self.left_bumper_rect, 8, 8)
        painter.setPen(self.COLORS['text'] if not self.buttons_state[4] else self.COLORS['text_dark'])
        painter.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        painter.drawText(self.left_bumper_rect, Qt.AlignmentFlag.AlignCenter, "LB")

        # RB (index 5)
        color = self.COLORS['button_active'] if self.buttons_state[5] else self.COLORS['button_inactive']
        painter.setBrush(color)
        painter.setPen(QPen(self.COLORS['controller_edge'], 1))
        painter.drawRoundedRect(self.right_bumper_rect, 8, 8)
        painter.setPen(self.COLORS['text'] if not self.buttons_state[5] else self.COLORS['text_dark'])
        painter.drawText(self.right_bumper_rect, Qt.AlignmentFlag.AlignCenter, "RB")

    def _draw_dpad(self, painter):
        """Draw the D-Pad"""
        cx, cy = self.dpad_center.x(), self.dpad_center.y()
        s = self.dpad_size

        directions = {
            'Up': (0, -1, 6),     # axis 6, value 1
            'Down': (0, 1, 6),    # axis 6, value -1
            'Left': (-1, 0, 7),   # axis 7, value 1
            'Right': (1, 0, 7),   # axis 7, value -1
        }

        for name, (dx, dy, axis) in directions.items():
            # Check if this direction is active
            if axis == 6:
                active = (dy == -1 and self.axes[6] > 0.5) or (dy == 1 and self.axes[6] < -0.5)
            else:
                active = (dx == -1 and self.axes[7] > 0.5) or (dx == 1 and self.axes[7] < -0.5)

            color = self.COLORS['dpad_active'] if active else self.COLORS['dpad']

            # Draw button
            rect = QRectF(cx + dx * s - s/2, cy + dy * s - s/2, s, s)
            painter.setBrush(color)
            painter.setPen(QPen(self.COLORS['controller_edge'], 1))
            painter.drawRoundedRect(rect, 5, 5)

            # Draw arrow
            painter.setPen(QPen(self.COLORS['text'] if not active else self.COLORS['text_dark'], 2))
            arrow_map = {'Up': '↑', 'Down': '↓', 'Left': '←', 'Right': '→'}
            painter.setFont(QFont("Arial", 14))
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, arrow_map[name])

        # Center piece
        center_rect = QRectF(cx - s/2, cy - s/2, s, s)
        painter.setBrush(self.COLORS['dpad'])
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(center_rect)

    def _draw_abxy(self, painter):
        """Draw ABXY buttons"""
        button_map = {'A': 0, 'B': 1, 'X': 2, 'Y': 3}

        for name, pos in self.button_positions.items():
            idx = button_map[name]
            active = self.buttons_state[idx]

            # Button color
            base_color = self.COLORS[name]
            if active:
                color = base_color.lighter(130)
            else:
                color = base_color.darker(150)

            painter.setBrush(color)
            painter.setPen(QPen(base_color, 2))
            painter.drawEllipse(pos, self.button_radius, self.button_radius)

            # Label
            painter.setPen(self.COLORS['text'])
            painter.setFont(QFont("Arial", 14, QFont.Weight.Bold))
            painter.drawText(
                QRectF(pos.x() - self.button_radius, pos.y() - self.button_radius,
                       self.button_radius * 2, self.button_radius * 2),
                Qt.AlignmentFlag.AlignCenter, name
            )

    def _draw_center_buttons(self, painter):
        """Draw Back, Xbox, Start buttons"""
        button_indices = {'Back': 6, 'Xbox': 8, 'Start': 7}

        for name, rect in self.center_buttons.items():
            idx = button_indices[name]
            active = self.buttons_state[idx]
            color = self.COLORS['button_active'] if active else self.COLORS['button_inactive']

            if name == 'Xbox':
                # Xbox button is circular
                center = rect.center()
                radius = min(rect.width(), rect.height()) / 2
                painter.setBrush(color)
                painter.setPen(QPen(self.COLORS['controller_edge'], 1))
                painter.drawEllipse(center, radius, radius)
                painter.setPen(self.COLORS['text'] if not active else self.COLORS['text_dark'])
                painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
                painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, "X")
            else:
                painter.setBrush(color)
                painter.setPen(QPen(self.COLORS['controller_edge'], 1))
                painter.drawRoundedRect(rect, 5, 5)
                painter.setPen(self.COLORS['text'] if not active else self.COLORS['text_dark'])
                painter.setFont(QFont("Arial", 8))
                symbol = "⎙" if name == 'Back' else "≡"
                painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, symbol)

    def _draw_stick(self, painter, center, stick_pos, label, is_dragging):
        """Draw an analog stick"""
        # Outer circle (stick area)
        painter.setBrush(self.COLORS['stick_area'])
        painter.setPen(QPen(self.COLORS['controller_edge'], 2))
        painter.drawEllipse(center, self.stick_radius, self.stick_radius)

        # Grid lines for reference
        painter.setPen(QPen(QColor(60, 60, 60), 1, Qt.PenStyle.DotLine))
        painter.drawLine(
            int(center.x() - self.stick_radius), int(center.y()),
            int(center.x() + self.stick_radius), int(center.y())
        )
        painter.drawLine(
            int(center.x()), int(center.y() - self.stick_radius),
            int(center.x()), int(center.y() + self.stick_radius)
        )

        # Calculate knob position
        knob_x = center.x() + stick_pos.x() * (self.stick_radius - self.knob_radius)
        knob_y = center.y() + stick_pos.y() * (self.stick_radius - self.knob_radius)
        knob_center = QPointF(knob_x, knob_y)

        # Knob
        knob_color = self.COLORS['stick_knob_active'] if is_dragging else self.COLORS['stick_knob']
        painter.setBrush(knob_color)
        painter.setPen(QPen(self.COLORS['controller_edge'], 2))
        painter.drawEllipse(knob_center, self.knob_radius, self.knob_radius)

        # Value display
        painter.setPen(self.COLORS['text'])
        painter.setFont(QFont("Arial", 8))
        value_text = f"{label}: ({stick_pos.x():.2f}, {-stick_pos.y():.2f})"
        painter.drawText(
            QRectF(center.x() - self.stick_radius, center.y() + self.stick_radius + 5,
                   self.stick_radius * 2, 20),
            Qt.AlignmentFlag.AlignCenter, value_text
        )

    def _draw_l3_r3(self, painter):
        """Draw L3 and R3 buttons"""
        # L3 (index 9)
        color = self.COLORS['button_active'] if self.buttons_state[9] else self.COLORS['button_inactive']
        painter.setBrush(color)
        painter.setPen(QPen(self.COLORS['controller_edge'], 1))
        painter.drawRoundedRect(self.l3_rect, 5, 5)
        painter.setPen(self.COLORS['text'] if not self.buttons_state[9] else self.COLORS['text_dark'])
        painter.setFont(QFont("Arial", 9))
        painter.drawText(self.l3_rect, Qt.AlignmentFlag.AlignCenter, "L3")

        # R3 (index 10)
        color = self.COLORS['button_active'] if self.buttons_state[10] else self.COLORS['button_inactive']
        painter.setBrush(color)
        painter.setPen(QPen(self.COLORS['controller_edge'], 1))
        painter.drawRoundedRect(self.r3_rect, 5, 5)
        painter.setPen(self.COLORS['text'] if not self.buttons_state[10] else self.COLORS['text_dark'])
        painter.drawText(self.r3_rect, Qt.AlignmentFlag.AlignCenter, "R3")

    def _draw_reset_buttons(self, painter):
        """Draw reset buttons for sticks"""
        # Reset Left
        color = self.COLORS['reset_button_hover'] if self.hover_reset_left else self.COLORS['reset_button']
        painter.setBrush(color)
        painter.setPen(QPen(self.COLORS['controller_edge'], 1))
        painter.drawRoundedRect(self.reset_left_rect, 5, 5)
        painter.setPen(self.COLORS['text'])
        painter.setFont(QFont("Arial", 8))
        painter.drawText(self.reset_left_rect, Qt.AlignmentFlag.AlignCenter, "RST")

        # Reset Right
        color = self.COLORS['reset_button_hover'] if self.hover_reset_right else self.COLORS['reset_button']
        painter.setBrush(color)
        painter.setPen(QPen(self.COLORS['controller_edge'], 1))
        painter.drawRoundedRect(self.reset_right_rect, 5, 5)
        painter.setPen(self.COLORS['text'])
        painter.drawText(self.reset_right_rect, Qt.AlignmentFlag.AlignCenter, "RST")

    def _point_in_circle(self, point, center, radius):
        """Check if point is inside circle"""
        dx = point.x() - center.x()
        dy = point.y() - center.y()
        return (dx * dx + dy * dy) <= (radius * radius)

    def _point_in_rect(self, point, rect):
        """Check if point is inside rectangle"""
        return rect.contains(point)

    def mousePressEvent(self, event):
        pos = event.position()

        # Check sticks
        if self._point_in_circle(pos, self.left_stick_center, self.stick_radius):
            self.dragging_left_stick = True
            self._update_stick_position(pos, 'left')

        elif self._point_in_circle(pos, self.right_stick_center, self.stick_radius):
            self.dragging_right_stick = True
            self._update_stick_position(pos, 'right')

        # Check triggers
        elif self._point_in_rect(pos, self.left_trigger_rect):
            self.dragging_left_trigger = True
            self._update_trigger_value(pos, 'left')

        elif self._point_in_rect(pos, self.right_trigger_rect):
            self.dragging_right_trigger = True
            self._update_trigger_value(pos, 'right')

        # Check reset buttons
        elif self._point_in_rect(pos, self.reset_left_rect):
            self.left_stick = QPointF(0, 0)

        elif self._point_in_rect(pos, self.reset_right_rect):
            self.right_stick = QPointF(0, 0)

        # Check bumpers
        elif self._point_in_rect(pos, self.left_bumper_rect):
            self.buttons_state[4] = 1 - self.buttons_state[4]

        elif self._point_in_rect(pos, self.right_bumper_rect):
            self.buttons_state[5] = 1 - self.buttons_state[5]

        # Check ABXY
        else:
            button_map = {'A': 0, 'B': 1, 'X': 2, 'Y': 3}
            for name, btn_pos in self.button_positions.items():
                if self._point_in_circle(pos, btn_pos, self.button_radius):
                    idx = button_map[name]
                    self.buttons_state[idx] = 1 - self.buttons_state[idx]
                    break

            # Check D-Pad
            cx, cy = self.dpad_center.x(), self.dpad_center.y()
            s = self.dpad_size
            dpad_regions = {
                'Up': QRectF(cx - s/2, cy - s*1.5, s, s),
                'Down': QRectF(cx - s/2, cy + s/2, s, s),
                'Left': QRectF(cx - s*1.5, cy - s/2, s, s),
                'Right': QRectF(cx + s/2, cy - s/2, s, s),
            }
            for name, rect in dpad_regions.items():
                if self._point_in_rect(pos, rect):
                    if name == 'Up':
                        self.axes[6] = 1.0 if self.axes[6] <= 0 else 0.0
                    elif name == 'Down':
                        self.axes[6] = -1.0 if self.axes[6] >= 0 else 0.0
                    elif name == 'Left':
                        self.axes[7] = 1.0 if self.axes[7] <= 0 else 0.0
                    elif name == 'Right':
                        self.axes[7] = -1.0 if self.axes[7] >= 0 else 0.0
                    break

            # Check center buttons
            button_indices = {'Back': 6, 'Xbox': 8, 'Start': 7}
            for name, rect in self.center_buttons.items():
                if name == 'Xbox':
                    center = rect.center()
                    radius = min(rect.width(), rect.height()) / 2
                    if self._point_in_circle(pos, center, radius):
                        idx = button_indices[name]
                        self.buttons_state[idx] = 1 - self.buttons_state[idx]
                        break
                else:
                    if self._point_in_rect(pos, rect):
                        idx = button_indices[name]
                        self.buttons_state[idx] = 1 - self.buttons_state[idx]
                        break

            # Check L3, R3
            if self._point_in_rect(pos, self.l3_rect):
                self.buttons_state[9] = 1 - self.buttons_state[9]
            elif self._point_in_rect(pos, self.r3_rect):
                self.buttons_state[10] = 1 - self.buttons_state[10]

        self.update()

    def mouseMoveEvent(self, event):
        pos = event.position()

        # Update stick positions while dragging
        if self.dragging_left_stick:
            self._update_stick_position(pos, 'left')
        elif self.dragging_right_stick:
            self._update_stick_position(pos, 'right')

        # Update trigger values while dragging
        if self.dragging_left_trigger:
            self._update_trigger_value(pos, 'left')
        elif self.dragging_right_trigger:
            self._update_trigger_value(pos, 'right')

        # Update hover states for reset buttons
        self.hover_reset_left = self._point_in_rect(pos, self.reset_left_rect)
        self.hover_reset_right = self._point_in_rect(pos, self.reset_right_rect)

        self.update()

    def mouseReleaseEvent(self, event):
        # Stop dragging but DON'T reset stick position (sticky behavior)
        self.dragging_left_stick = False
        self.dragging_right_stick = False
        self.dragging_left_trigger = False
        self.dragging_right_trigger = False
        self.update()

    def _update_stick_position(self, pos, stick):
        """Update stick position from mouse position"""
        if stick == 'left':
            center = self.left_stick_center
        else:
            center = self.right_stick_center

        # Calculate relative position
        dx = pos.x() - center.x()
        dy = pos.y() - center.y()

        # Normalize to -1 to 1 range
        max_dist = self.stick_radius - self.knob_radius
        x = max(-1, min(1, dx / max_dist))
        y = max(-1, min(1, dy / max_dist))

        # Clamp to circle
        dist = math.sqrt(x * x + y * y)
        if dist > 1:
            x /= dist
            y /= dist

        if stick == 'left':
            self.left_stick = QPointF(x, y)
        else:
            self.right_stick = QPointF(x, y)

    def _update_trigger_value(self, pos, trigger):
        """Update trigger value from mouse position"""
        if trigger == 'left':
            rect = self.left_trigger_rect
        else:
            rect = self.right_trigger_rect

        # Calculate value from horizontal position
        value = (pos.x() - rect.x()) / rect.width()
        value = max(0, min(1, value))

        if trigger == 'left':
            self.left_trigger = value
        else:
            self.right_trigger = value

    def publish_joy(self):
        # Update axes from stick positions
        self.axes[0] = -self.left_stick.x()   # Invert X: left=+1, right=-1
        self.axes[1] = -self.left_stick.y()  # Invert Y: up=+1, down=-1
        self.axes[2] = self.left_trigger
        self.axes[3] = -self.right_stick.x()  # Invert X: left=+1, right=-1
        self.axes[4] = -self.right_stick.y()  # Invert Y: up=+1, down=-1
        self.axes[5] = self.right_trigger

        msg = Joy()
        msg.axes = self.axes
        msg.buttons = self.buttons_state
        self.joy_pub.publish(msg)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = VirtualXbox()

    signal.signal(signal.SIGINT, lambda *args: app.quit())

    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    node.show()
    app.exec()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
