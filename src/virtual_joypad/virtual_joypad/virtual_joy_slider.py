import sys
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QSlider, QLabel, QLineEdit
)
from PyQt6.QtCore import Qt, QTimer

FREQ = 10

class VirtualXbox(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "virtual_xbox")
        QWidget.__init__(self)

        self.joy_pub = self.create_publisher(Joy, "joy", 10)
        self.setWindowTitle("ROS2 Virtual Xbox Controller")
        self.setGeometry(100, 100, 400, 600)

        layout = QVBoxLayout()
        self.slider_inputs = {}  # Store slider and input field pairs

        stick_definitions = {
            "Left X": Qt.Orientation.Horizontal,
            "Left Y": Qt.Orientation.Vertical,
            "Right X": Qt.Orientation.Horizontal,
            "Right Y": Qt.Orientation.Vertical,
            "LT": Qt.Orientation.Horizontal,
            "RT": Qt.Orientation.Horizontal,
        }

        for name, orientation in stick_definitions.items():
            if orientation == Qt.Orientation.Vertical:
                vbox = QVBoxLayout()
                label = QLabel(name)

                slider = QSlider(orientation)
                slider.setRange(-100, 100)
                slider.setValue(0)

                input_field = QLineEdit("0")
                input_field.setFixedWidth(50)

                slider.valueChanged.connect(lambda val, inp=input_field: inp.setText(str(val)))
                input_field.editingFinished.connect(
                    lambda inp=input_field, s=slider: s.setValue(self.safe_int(inp.text()))
                )

                vbox.addWidget(label, alignment=Qt.AlignmentFlag.AlignHCenter)
                vbox.addWidget(slider, alignment=Qt.AlignmentFlag.AlignHCenter)
                vbox.addWidget(input_field, alignment=Qt.AlignmentFlag.AlignHCenter)

                layout.addLayout(vbox)
            else:
                hbox = QHBoxLayout()
                label = QLabel(name)

                slider = QSlider(orientation)
                slider.setRange(-100, 100)
                slider.setValue(0)
                slider.setInvertedAppearance(True)

                input_field = QLineEdit("0")
                input_field.setFixedWidth(50)

                slider.valueChanged.connect(lambda val, inp=input_field: inp.setText(str(val)))
                input_field.editingFinished.connect(
                    lambda inp=input_field, s=slider: s.setValue(self.safe_int(inp.text()))
                )

                hbox.addWidget(label)
                hbox.addWidget(slider)
                hbox.addWidget(input_field)
                layout.addLayout(hbox)

            self.slider_inputs[name] = (slider, input_field)


        reset_button = QPushButton("Reset Sticks")
        reset_button.clicked.connect(self.reset_sticks)
        layout.addWidget(reset_button)

        self.buttons = []
        self.buttons_state = [0] * 11
        button_names = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start", "Xbox", "L Stick", "R Stick"]
        button_layout = QHBoxLayout()
        for i, name in enumerate(button_names):
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _, idx=i: self.set_button(idx))
            self.buttons.append(btn)
            button_layout.addWidget(btn)
        layout.addLayout(button_layout)

        # D-Pad
        self.dpad = {
            "Up": QPushButton("↑"),
            "Down": QPushButton("↓"),
            "Left": QPushButton("←"),
            "Right": QPushButton("→"),
        }
        self.axes = [0.0] * 8  # 6 axes + 2 dpad
        dpad_layout = QHBoxLayout()
        for key, btn in self.dpad.items():
            btn.setCheckable(True)
            btn.clicked.connect(lambda _, key=key: self.set_dpad(key))
            dpad_layout.addWidget(btn)
        layout.addLayout(dpad_layout)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_joy)
        self.timer.start(int(1000 // FREQ))

    def safe_int(self, val):
        try:
            return max(-100, min(100, int(val)))
        except ValueError:
            return 0

    def reset_sticks(self):
        for slider, line_edit in self.slider_inputs.values():
            slider.setValue(0)
            line_edit.setText("0")

    def set_button(self, index):
        self.buttons_state[index] = 1 if self.buttons[index].isChecked() else 0

    def set_dpad(self, key):
        dpad_map = {"Up": 6, "Down": 6, "Left": 7, "Right": 7}
        values = {"Up": 1.0, "Down": -1.0, "Left": -1.0, "Right": 1.0}

        self.axes[dpad_map[key]] = values[key] if self.dpad[key].isChecked() else 0.0

    def publish_joy(self):
        self.axes[0] = -self.slider_inputs["Left X"][0].value() / 100.0   # left=+1, right=-1
        self.axes[1] = self.slider_inputs["Left Y"][0].value() / 100.0
        self.axes[2] = self.slider_inputs["LT"][0].value() / 100.0
        self.axes[3] = -self.slider_inputs["Right X"][0].value() / 100.0  # left=+1, right=-1
        self.axes[4] = self.slider_inputs["Right Y"][0].value() / 100.0
        self.axes[5] = self.slider_inputs["RT"][0].value() / 100.0

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
