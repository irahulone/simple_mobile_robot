"""Virtual Robot Arm Controller with PyQt6 GUI for ROS2."""

import sys
import signal
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QSlider, QLabel, QLineEdit, QTabWidget, QTextEdit, QGroupBox,
    QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPalette, QColor

from .config_loader import ControllerConfig


class AxisControl(QWidget):
    """Widget for controlling a single axis with slider, input field, and reset button."""

    def __init__(self, name, min_val, max_val, resolution=None, parent=None):
        super().__init__(parent)
        # Validate range to prevent division by zero
        if min_val >= max_val:
            raise ValueError(f"Invalid range for axis '{name}': min_val ({min_val}) must be less than max_val ({max_val})")

        self.name = name
        self.min_val = min_val
        self.max_val = max_val
        self.resolution = resolution  # Minimum step size (e.g., 0.001 for meters)

        # Calculate decimal places based on resolution
        if self.resolution is not None and self.resolution > 0:
            # Calculate number of decimal places needed for this resolution
            # e.g., 0.1 -> 1, 0.01 -> 2, 0.001 -> 3, 1.0 -> 0
            self.decimal_places = max(0, -int(math.floor(math.log10(self.resolution))))
        else:
            self.decimal_places = 3  # Default to 3 decimal places

        layout = QHBoxLayout()
        layout.setContentsMargins(5, 2, 5, 2)

        # Label
        self.label = QLabel(f"{name}:")
        self.label.setMinimumWidth(80)
        layout.addWidget(self.label)

        # Slider area with min/max labels below
        slider_container = QVBoxLayout()
        slider_container.setSpacing(0)

        # Slider (use range -1000 to 1000 for smooth control)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(-1000, 1000)
        self.slider.setValue(0)
        self.slider.setMinimumWidth(200)
        self.slider.valueChanged.connect(self._on_slider_changed)

        # Set keyboard single step based on resolution
        if self.resolution is not None and self.resolution > 0:
            # Calculate how many slider units correspond to one resolution step
            value_range = self.max_val - self.min_val
            slider_range = 2000  # -1000 to 1000
            single_step = int((self.resolution / value_range) * slider_range)
            if single_step < 1:
                single_step = 1
            self.slider.setSingleStep(single_step)

        slider_container.addWidget(self.slider)

        # Min/Max labels below slider
        minmax_layout = QHBoxLayout()
        minmax_layout.setContentsMargins(0, 0, 0, 0)
        self.min_label = QLabel(f"{min_val:.1f}")
        self.min_label.setStyleSheet("font-size: 9pt; color: gray;")
        self.min_label.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self.max_label = QLabel(f"{max_val:.1f}")
        self.max_label.setStyleSheet("font-size: 9pt; color: gray;")
        self.max_label.setAlignment(Qt.AlignmentFlag.AlignRight)

        minmax_layout.addWidget(self.min_label)
        minmax_layout.addStretch()
        minmax_layout.addWidget(self.max_label)
        slider_container.addLayout(minmax_layout)

        layout.addLayout(slider_container)

        # Input field
        initial_value = f"{0:.{self.decimal_places}f}"
        self.input_field = QLineEdit(initial_value)
        self.input_field.setFixedWidth(80)
        self.input_field.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.input_field.editingFinished.connect(self._on_input_changed)
        layout.addWidget(self.input_field)

        # Unit label
        self.unit_label = QLabel("")
        self.unit_label.setMinimumWidth(30)
        layout.addWidget(self.unit_label)

        # Reset button
        self.reset_button = QPushButton("Reset")
        self.reset_button.setFixedWidth(60)
        self.reset_button.clicked.connect(self.reset)
        layout.addWidget(self.reset_button)

        self.setLayout(layout)

    def set_unit_label(self, unit):
        """Set the unit label text."""
        self.unit_label.setText(unit)
        # Update min/max labels with unit using calculated decimal places
        if unit:
            self.min_label.setText(f"{self.min_val:.{self.decimal_places}f} {unit}")
            self.max_label.setText(f"{self.max_val:.{self.decimal_places}f} {unit}")
        else:
            self.min_label.setText(f"{self.min_val:.{self.decimal_places}f}")
            self.max_label.setText(f"{self.max_val:.{self.decimal_places}f}")

    def _on_slider_changed(self, value):
        """Handle slider value change."""
        # Convert slider value (-1000 to 1000) to actual range
        actual_value = self._slider_to_value(value)
        self.input_field.setText(f"{actual_value:.{self.decimal_places}f}")

    def _on_input_changed(self):
        """Handle input field change."""
        try:
            value = float(self.input_field.text())
            value = max(self.min_val, min(self.max_val, value))
            slider_value = self._value_to_slider(value)
            self.slider.setValue(slider_value)
            self.input_field.setText(f"{value:.{self.decimal_places}f}")
        except ValueError:
            # Invalid input, reset to current slider value
            self._on_slider_changed(self.slider.value())

    def _slider_to_value(self, slider_value):
        """Convert slider position to actual value."""
        # Map -1000..1000 to min_val..max_val
        normalized = slider_value / 1000.0  # -1.0 to 1.0
        range_size = self.max_val - self.min_val
        mid_point = (self.max_val + self.min_val) / 2.0
        value = mid_point + normalized * range_size / 2.0

        # Round to resolution if specified
        if self.resolution is not None and self.resolution > 0:
            value = round(value / self.resolution) * self.resolution

        return value

    def _value_to_slider(self, value):
        """Convert actual value to slider position."""
        range_size = self.max_val - self.min_val
        mid_point = (self.max_val + self.min_val) / 2.0
        normalized = (value - mid_point) / (range_size / 2.0)
        return int(normalized * 1000.0)

    def get_value(self):
        """Get current value (already rounded to resolution in _slider_to_value)."""
        return self._slider_to_value(self.slider.value())

    def set_value(self, value):
        """Set value programmatically."""
        value = max(self.min_val, min(self.max_val, value))
        slider_value = self._value_to_slider(value)
        self.slider.setValue(slider_value)

    def reset(self):
        """Reset to zero."""
        self.set_value(0.0)

    def set_enabled(self, enabled):
        """Enable or disable the axis control."""
        self.slider.setEnabled(enabled)
        self.input_field.setEnabled(enabled)
        self.reset_button.setEnabled(enabled)


class VirtualController(Node, QWidget):
    """Main Virtual Robot Arm Controller application."""

    def __init__(self, config):
        Node.__init__(self, "virtual_controller")
        QWidget.__init__(self)

        self.config = config
        self.robot_enabled = False

        # Setup window
        self.setWindowTitle("Virtual Robot Arm Controller")
        # Window size will be adjusted after GUI setup

        # Create publishers
        self._setup_publishers()

        # Create GUI
        self._setup_gui()

        # Setup timer for publishing
        self.timer = QTimer()
        self.timer.timeout.connect(self._publish_messages)
        interval_ms = int(1000.0 / self.config.get_frequency())
        self.timer.start(interval_ms)

        # Set initial enabled state (disabled by default)
        self._update_axes_enabled()

        # Set initial home button state (disabled in Joint mode)
        self._update_home_button_state()

        # Adjust window size based on content
        self._adjust_window_size()

        self.get_logger().info("Virtual Controller started")

    def _setup_publishers(self):
        """Setup ROS2 publishers."""
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            self.config.get_joint_command_topic(),
            10
        )
        self.cartesian_pub = self.create_publisher(
            Twist,
            self.config.get_cartesian_command_topic(),
            10
        )
        self.joint_enable_pub = self.create_publisher(
            Bool,
            self.config.get_joint_enable_topic(),
            10
        )
        self.cartesian_enable_pub = self.create_publisher(
            Bool,
            self.config.get_cartesian_enable_topic(),
            10
        )
        self.robot_enable_pub = self.create_publisher(
            Bool,
            self.config.get_robot_enable_topic(),
            10
        )

    def _setup_gui(self):
        """Setup the GUI layout."""
        main_layout = QVBoxLayout()

        # Create tab widget for Joint and Cartesian modes
        self.tabs = QTabWidget()
        self.tabs.currentChanged.connect(self._on_tab_changed)

        # Joint mode tab
        self.joint_tab = self._create_joint_tab()
        self.tabs.addTab(self.joint_tab, "Joint Mode")

        # Cartesian mode tab
        self.cartesian_tab = self._create_cartesian_tab()
        self.tabs.addTab(self.cartesian_tab, "Cartesian Mode")

        # Tab widget should not expand vertically
        self.tabs.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        main_layout.addWidget(self.tabs)

        # Control panel
        control_panel = self._create_control_panel()
        main_layout.addWidget(control_panel)

        # Configuration display (this will expand)
        config_display = self._create_config_display()
        main_layout.addWidget(config_display)

        self.setLayout(main_layout)

    def _create_joint_tab(self):
        """Create Joint mode tab."""
        widget = QWidget()
        layout = QVBoxLayout()

        # Title and Reset All button in the same row
        title_layout = QHBoxLayout()
        title = QLabel("Joint Mode Control")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        title_layout.addWidget(title)

        title_layout.addStretch()

        self.joint_reset_all_button = QPushButton("Reset All")
        self.joint_reset_all_button.setFixedWidth(100)
        self.joint_reset_all_button.setFixedHeight(30)
        self.joint_reset_all_button.setStyleSheet("font-weight: bold;")
        self.joint_reset_all_button.setToolTip("Reset all joint axes to 0 degrees")
        self.joint_reset_all_button.clicked.connect(self._on_joint_reset_all_clicked)
        title_layout.addWidget(self.joint_reset_all_button)

        layout.addLayout(title_layout)

        # Create n joint axes (from config)
        self.joint_axes = []
        num_joints = self.config.get_num_joints()
        min_angle = self.config.get_joint_min_angle()
        max_angle = self.config.get_joint_max_angle()
        joint_resolution = self.config.get_joint_resolution()

        for i in range(num_joints):
            axis = AxisControl(f"Joint {i+1}", min_angle, max_angle, resolution=joint_resolution)
            axis.set_unit_label("deg")
            self.joint_axes.append(axis)
            layout.addWidget(axis)

        # Push all content to the top (no vertical expansion)
        layout.addStretch()
        widget.setLayout(layout)

        # Set size policy to not expand vertically
        widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        return widget

    def _create_cartesian_tab(self):
        """Create Cartesian mode tab."""
        widget = QWidget()
        layout = QVBoxLayout()

        # Title
        title = QLabel("Cartesian Mode Control")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        layout.addWidget(title)

        # Position axes (X, Y, Z)
        position_group = QGroupBox("Position")
        position_layout = QVBoxLayout()

        self.cartesian_position_axes = []
        # Config values are in meters
        min_pos_m = self.config.get_cartesian_position_min()
        max_pos_m = self.config.get_cartesian_position_max()
        pos_resolution = self.config.get_cartesian_position_resolution()

        for name in ["X", "Y", "Z"]:
            axis = AxisControl(name, min_pos_m, max_pos_m, resolution=pos_resolution)
            axis.set_unit_label("m")
            self.cartesian_position_axes.append(axis)
            position_layout.addWidget(axis)

        position_group.setLayout(position_layout)
        layout.addWidget(position_group)

        # Orientation axes (Roll, Pitch, Yaw)
        orientation_group = QGroupBox("Orientation")
        orientation_layout = QVBoxLayout()

        self.cartesian_orientation_axes = []
        min_ori = self.config.get_cartesian_orientation_min()
        max_ori = self.config.get_cartesian_orientation_max()
        ori_resolution = self.config.get_cartesian_orientation_resolution()

        for name in ["Roll", "Pitch", "Yaw"]:
            axis = AxisControl(name, min_ori, max_ori, resolution=ori_resolution)
            axis.set_unit_label("deg")
            self.cartesian_orientation_axes.append(axis)
            orientation_layout.addWidget(axis)

        orientation_group.setLayout(orientation_layout)
        layout.addWidget(orientation_group)

        # Push all content to the top (no vertical expansion)
        layout.addStretch()
        widget.setLayout(layout)

        # Set size policy to not expand vertically
        widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        return widget

    def _create_control_panel(self):
        """Create control panel with Enable/Disable and Home buttons."""
        group = QGroupBox("Control Panel")
        layout = QHBoxLayout()

        # Enable/Disable button
        self.enable_button = QPushButton("Robot: DISABLED")
        self.enable_button.setCheckable(True)
        self.enable_button.setFixedHeight(50)
        self.enable_button.setToolTip("Click to toggle robot control on/off")
        self.enable_button.clicked.connect(self._on_enable_clicked)
        self._update_enable_button_style()
        layout.addWidget(self.enable_button)

        # Home position button (Cartesian only)
        self.home_button = QPushButton("Home Position\n(Cartesian Only)")
        self.home_button.setFixedHeight(50)
        self.home_button.setToolTip("Reset Cartesian axes to configured home position\n(Only available in Cartesian mode)")
        self.home_button.clicked.connect(self._on_home_clicked)
        layout.addWidget(self.home_button)

        group.setLayout(layout)

        # Set size policy to not expand vertically
        group.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        return group

    def _create_config_display(self):
        """Create configuration display area."""
        self.config_text = QTextEdit()
        self.config_text.setReadOnly(True)
        self.config_text.setMinimumHeight(150)  # Minimum height instead of maximum
        self.config_text.setFrameStyle(0)  # Remove frame
        self.config_text.setStyleSheet("background-color: transparent; border: none;")
        self.config_text.setPlainText(self.config.get_config_display_string())

        # Set size policy to expand vertically when window is resized
        self.config_text.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        return self.config_text

    def _on_tab_changed(self, index):
        """Handle tab change."""
        # Check if UI is fully initialized (may be called during setup)
        if not hasattr(self, 'enable_button'):
            return

        # Disable robot if configured to do so on tab switch
        if self.config.get_disable_on_tab_switch():
            if self.robot_enabled:
                self.robot_enabled = False
                self.enable_button.setChecked(False)
                self._update_enable_button_style()
                self.get_logger().info("Robot automatically disabled on tab switch")

        # Update axis enable state for the new tab
        self._update_axes_enabled()
        # Update home button state (only enabled in Cartesian mode)
        self._update_home_button_state()

    def _on_enable_clicked(self):
        """Handle Enable/Disable button click."""
        self.robot_enabled = self.enable_button.isChecked()
        self._update_enable_button_style()
        self._update_axes_enabled()

    def _update_enable_button_style(self):
        """Update Enable/Disable button appearance."""
        if self.robot_enabled:
            self.enable_button.setText("Robot: ENABLED")
            self.enable_button.setStyleSheet(
                "background-color: #4CAF50; color: white; font-weight: bold; font-size: 12pt;"
            )
        else:
            self.enable_button.setText("Robot: DISABLED")
            self.enable_button.setStyleSheet(
                "background-color: #757575; color: white; font-weight: bold; font-size: 12pt;"
            )

    def _update_axes_enabled(self):
        """Update axis controls enabled state based on robot enable state."""
        # Check if axes exist (may not exist during initialization)
        if not hasattr(self, 'joint_axes') or not hasattr(self, 'cartesian_position_axes'):
            return

        current_tab = self.tabs.currentIndex()

        if current_tab == 0:
            # Joint mode - enable/disable joint axes
            for axis in self.joint_axes:
                axis.set_enabled(self.robot_enabled)
            # Enable/disable Reset All button
            if hasattr(self, 'joint_reset_all_button'):
                self.joint_reset_all_button.setEnabled(self.robot_enabled)
        else:
            # Cartesian mode - enable/disable cartesian axes
            for axis in self.cartesian_position_axes:
                axis.set_enabled(self.robot_enabled)
            for axis in self.cartesian_orientation_axes:
                axis.set_enabled(self.robot_enabled)

    def _adjust_window_size(self):
        """Adjust window size based on number of axes."""
        # Calculate number of axes for each mode
        num_joint_axes = len(self.joint_axes)
        num_cartesian_axes = 6  # Always 6 (3 position + 3 orientation)

        # Use the larger number to determine window height
        max_axes = max(num_joint_axes, num_cartesian_axes)

        # Base height + height per axis
        base_height = 350  # Title, tabs, control panel, config display
        axis_height = 55   # Height per axis control (including spacing)

        window_height = base_height + (max_axes * axis_height)

        # Set window geometry
        self.setGeometry(100, 100, 800, window_height)

    def closeEvent(self, event):
        """Handle window close event."""
        self.get_logger().info("Closing Virtual Controller...")
        event.accept()

    def _update_home_button_state(self):
        """Update Home Position button enabled state based on current tab."""
        # Check if home_button exists (may not exist during initialization)
        if not hasattr(self, 'home_button'):
            return

        current_tab = self.tabs.currentIndex()
        # Home Position is only available in Cartesian mode (tab index 1)
        self.home_button.setEnabled(current_tab == 1)

    def _on_home_clicked(self):
        """Handle Home Position button click (Cartesian mode only)."""
        home = self.config.get_home_position()

        # Only works in Cartesian mode
        if self.tabs.currentIndex() == 1:
            # Cartesian mode - reset to home position
            # Position values in meters (same as GUI), orientation in degrees
            self.cartesian_position_axes[0].set_value(home['x'])
            self.cartesian_position_axes[1].set_value(home['y'])
            self.cartesian_position_axes[2].set_value(home['z'])
            self.cartesian_orientation_axes[0].set_value(home['roll'])
            self.cartesian_orientation_axes[1].set_value(home['pitch'])
            self.cartesian_orientation_axes[2].set_value(home['yaw'])

    def _on_joint_reset_all_clicked(self):
        """Handle Reset All Joints button click."""
        # Reset all joint axes to 0
        for axis in self.joint_axes:
            axis.reset()

    def _publish_messages(self):
        """Publish ROS2 messages at configured frequency."""
        try:
            current_tab = self.tabs.currentIndex()
            joint_mode_active = (current_tab == 0)

            # Publish enable flags
            joint_enable_msg = Bool()
            cartesian_enable_msg = Bool()
            robot_enable_msg = Bool()

            if self.robot_enabled:
                joint_enable_msg.data = joint_mode_active
                cartesian_enable_msg.data = not joint_mode_active
                robot_enable_msg.data = True
            else:
                joint_enable_msg.data = False
                cartesian_enable_msg.data = False
                robot_enable_msg.data = False

            self.joint_enable_pub.publish(joint_enable_msg)
            self.cartesian_enable_pub.publish(cartesian_enable_msg)
            self.robot_enable_pub.publish(robot_enable_msg)

            # Only publish command messages if enabled
            if not self.robot_enabled:
                return

            if joint_mode_active:
                self._publish_joint_command()
            else:
                self._publish_cartesian_command()

        except Exception as e:
            self.get_logger().error(f"Publishing failed: {e}")
            # Auto-disable robot on publishing error for safety
            if self.robot_enabled:
                self.robot_enabled = False
                self.enable_button.setChecked(False)
                self._update_enable_button_style()
                self._update_axes_enabled()
                self.get_logger().warning("Robot automatically disabled due to publishing error")

    def _publish_joint_command(self):
        """Publish joint command message."""
        try:
            msg = Float64MultiArray()

            # Get values from GUI (in degrees) and convert to publish unit
            values = []
            for axis in self.joint_axes:
                deg_value = axis.get_value()
                pub_value = self.config.joint_angle_to_publish_unit(deg_value)
                values.append(pub_value)

            msg.data = values
            self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint command: {e}")
            raise  # Re-raise to be caught by _publish_messages

    def _publish_cartesian_command(self):
        """Publish cartesian command message."""
        try:
            msg = Twist()

            # Position (GUI values are in meters, convert to publish unit)
            pos_x_m = self.cartesian_position_axes[0].get_value()
            pos_y_m = self.cartesian_position_axes[1].get_value()
            pos_z_m = self.cartesian_position_axes[2].get_value()

            msg.linear.x = self.config.cartesian_position_to_publish_unit(pos_x_m)
            msg.linear.y = self.config.cartesian_position_to_publish_unit(pos_y_m)
            msg.linear.z = self.config.cartesian_position_to_publish_unit(pos_z_m)

            # Orientation (in degrees from GUI, convert to publish unit)
            roll = self.cartesian_orientation_axes[0].get_value()
            pitch = self.cartesian_orientation_axes[1].get_value()
            yaw = self.cartesian_orientation_axes[2].get_value()

            msg.angular.x = self.config.cartesian_orientation_to_publish_unit(roll)
            msg.angular.y = self.config.cartesian_orientation_to_publish_unit(pitch)
            msg.angular.z = self.config.cartesian_orientation_to_publish_unit(yaw)

            self.cartesian_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish cartesian command: {e}")
            raise  # Re-raise to be caught by _publish_messages


def main(args=None):
    """Main entry point."""
    controller = None
    timer = None
    app = None

    try:
        # Initialize ROS2
        rclpy.init(args=args)

        # Load configuration
        config = ControllerConfig()

        # Create Qt application
        app = QApplication(sys.argv)

        # Setup signal handler for Ctrl+C
        # Note: Signal handlers should only call async-signal-safe functions
        # We use a minimal handler that only calls app.quit()
        def signal_handler(sig, frame):
            """Handle Ctrl+C gracefully.

            This handler is kept minimal to be as safe as possible.
            It only calls QApplication.quit() which is designed to be
            called from signal handlers.
            """
            if app:
                app.quit()

        signal.signal(signal.SIGINT, signal_handler)

        # Create and show controller
        controller = VirtualController(config)
        controller.show()

        # Create a timer to allow Python signal handlers to run
        # This is necessary for Ctrl+C to work with Qt event loop
        timer = QTimer()
        timer.timeout.connect(lambda: None)  # Do nothing, just let Python process signals
        timer.start(100)  # Check every 100ms

        # Run Qt event loop
        app.exec()

    except Exception as e:
        print(f"Error during initialization or execution: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        raise

    finally:
        # Ensure proper cleanup regardless of how we exit
        if timer:
            timer.stop()
        if controller:
            try:
                controller.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}", file=sys.stderr)
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error shutting down ROS2: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
