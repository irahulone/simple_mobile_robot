# Virtual Robot Arm Controller - Detailed Specification

## 1. Overview

This application provides a virtual controller for robot arm manipulation through a PyQt6-based GUI. The controller supports two operation modes (Joint and Cartesian) and publishes control commands via ROS2 topics.

## 2. System Requirements

### 2.1 Software Dependencies
- Python 3.8+
- ROS2 (Humble or later)
- PyQt6
- rclpy

### 2.2 ROS2 Message Types
- `std_msgs/Float64MultiArray` - Joint mode commands
- `geometry_msgs/Twist` - Cartesian mode commands
- `std_msgs/Bool` - Enable/Disable signals

## 3. Functional Requirements

### 3.1 Operation Modes

#### 3.1.1 Joint Mode
- Controls n independent joint angles (configurable, default: 6)
- Number of joints: 1-12 (configurable via `num_joints` parameter)
- Range: -90° to +90° for each joint (configurable)
- Publishes to: `/joint_cmd` (default, configurable)
- Message type: `std_msgs/Float64MultiArray`

#### 3.1.2 Cartesian Mode
- Controls position (x, y, z) and orientation (roll, pitch, yaw)
- Position range: -0.10 m to +0.10 m
- Orientation range: -30° to +30°
- Publishes to: `/cart_cmd` (default, configurable)
- Message type: `geometry_msgs/Twist`
  - linear.x, linear.y, linear.z for position
  - angular.x, angular.y, angular.z for orientation

### 3.2 Mode Switching
- UI element: Tab-based interface
- Two tabs: "Joint Mode" and "Cartesian Mode"
- **Safety feature**: Robot is automatically disabled when switching tabs (configurable)
  - Can be disabled by setting `safety.disable_on_tab_switch: false` in config
  - Default: `true` (automatically disable on tab switch for safety)

### 3.3 Control Axes

Each mode has 6 controllable axes with the following UI elements per axis:
- Label (axis name)
- Slider (continuous value adjustment)
- Numeric input field (direct value entry)
- Individual Reset button (resets that axis to 0)

#### 3.3.1 Joint Mode Axes
- Joint 1, Joint 2, Joint 3, Joint 4, Joint 5, Joint 6

#### 3.3.2 Cartesian Mode Axes
- Position: X, Y, Z
- Orientation: Roll, Pitch, Yaw

### 3.4 Enable/Disable System

#### 3.4.1 Three Enable Signals
1. **Joint Enable** (`/joint_en`)
   - Type: `std_msgs/Bool`
   - True: Joint mode is active
   - False: Joint mode is inactive

2. **Cartesian Enable** (`/cart_en`)
   - Type: `std_msgs/Bool`
   - True: Cartesian mode is active
   - False: Cartesian mode is inactive

3. **Robot Enable** (`/enable`)
   - Type: `std_msgs/Bool`
   - True: Robot is enabled
   - False: Robot is disabled (master disable)

#### 3.4.2 Enable Logic
- When Robot Enable is True:
  - Currently selected mode's enable flag is True
  - Other mode's enable flag is False
  - Command messages are published at configured frequency

- When Robot Enable is False (Disable):
  - Both Joint Enable and Cartesian Enable are False
  - No command messages are published (transmission completely stopped)
  - Enable state messages are still published

### 3.5 Control Panel

The control panel contains:

1. **Enable/Disable Button**
   - Toggle button for Robot Enable state
   - Visual feedback (color change when enabled)
   - Publishes to `/enable` topic

2. **Home Position Button**
   - **Only enabled in Cartesian mode**
   - Resets all Cartesian axes to configured home position values
   - Default home position: all axes to 0
   - Configurable via YAML file
   - Disabled (grayed out) in Joint mode

### 3.6 Configuration Display Area

GUI displays current configuration values:
- Topic names (Joint, Cartesian, enables)
- Publishing frequency
- Range limits (joint angles, cartesian position/orientation)
- Home position values
- Unit settings (degrees/radians for angles, cm/m for distances)

## 4. ROS2 Interface

### 4.1 Publishing Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_cmd` | `std_msgs/Float64MultiArray` | 10 Hz | Joint angle commands |
| `/cart_cmd` | `geometry_msgs/Twist` | 10 Hz | Cartesian velocity commands |
| `/joint_en` | `std_msgs/Bool` | 10 Hz | Joint mode enable flag |
| `/cart_en` | `std_msgs/Bool` | 10 Hz | Cartesian mode enable flag |
| `/enable` | `std_msgs/Bool` | 10 Hz | Master robot enable flag |

Note: All topic names are configurable via YAML configuration file.

### 4.2 Message Content

#### 4.2.1 Joint Mode Message
```python
# std_msgs/Float64MultiArray
data: [joint1, joint2, joint3, joint4, joint5, joint6]
```
Values are in the configured unit (default: radians, can be degrees)

#### 4.2.2 Cartesian Mode Message
```python
# geometry_msgs/Twist
linear:
  x: position_x  # in configured unit (default: meters)
  y: position_y
  z: position_z
angular:
  x: roll        # in configured unit (default: radians)
  y: pitch
  z: yaw
```

### 4.3 Publishing Behavior

- **Normal Operation**: Publishes at configured frequency (default: 10 Hz)
- **Disable State**: Command messages (joint_cmd, cart_cmd) are NOT published
- Enable flags are always published regardless of state

## 5. Configuration File Specification

### 5.1 File Location
- Path: `<package_path>/config/controller_config.yaml`
- Format: YAML
- The configuration file is located within the package directory structure
- Can be accessed using ROS2 package resource lookup

### 5.2 Configuration Structure

```yaml
# Topic names
topics:
  joint_command: "/joint_cmd"
  cartesian_command: "/cart_cmd"
  joint_enable: "/joint_en"
  cartesian_enable: "/cart_en"
  robot_enable: "/enable"

# Publishing frequency (Hz)
frequency: 10

# Safety settings
safety:
  disable_on_tab_switch: true  # Automatically disable robot when switching tabs

# Joint mode settings
joint:
  num_joints: 6     # number of joints (1-12)
  min_angle: -90    # degrees
  max_angle: 90     # degrees
  resolution: 0.1   # degrees (minimum step size)
  angle_unit: "rad"  # "deg" or "rad" - unit for publishing

# Cartesian mode settings
cartesian:
  position:
    min: -0.10      # meters
    max: 0.10       # meters
    resolution: 0.001  # meters (minimum step size)
    unit: "m"       # "m" or "cm" - unit for publishing
  orientation:
    min: -30        # degrees
    max: 30         # degrees
    resolution: 0.1 # degrees (minimum step size)
    unit: "rad"     # "deg" or "rad" - unit for publishing

# Home position (Cartesian coordinates)
home_position:
  x: 0.0
  y: 0.0
  z: 0.0
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### 5.3 Unit Conversion

The controller performs automatic unit conversion based on configuration:

**Angle Units:**
- GUI always displays in degrees
- If `angle_unit: "rad"`, converts to radians before publishing
- Conversion: radians = degrees × π / 180

**Distance Units:**
- If `unit: "cm"`, GUI values are in cm
- If `unit: "m"`, GUI values are converted from cm to m
- Conversion: meters = centimeters / 100

### 5.4 Default Values

When configuration file is not found or values are missing, use these defaults:
- Number of joints: 6
- Frequency: 10 Hz
- Disable on tab switch: true (enabled)
- Joint angles: -90° to 90°, publish in radians
- Cartesian position: -0.10 m to 0.10 m, publish in meters
- Cartesian orientation: -30° to 30°, publish in radians
- Home position: all zeros (meters for position, degrees for orientation)
- Topic names: `/joint_cmd`, `/cart_cmd`, `/joint_en`, `/cart_en`, `/enable`

## 6. UI/UX Design

### 6.1 Window Resize Behavior

When the user resizes the window:
- **Controller area** (Joint/Cartesian tabs): Fixed height, does not expand vertically
- **Control Panel**: Fixed height, does not expand vertically
- **Configuration Display**: Expands vertically to fill available space

This design ensures that:
- Axis controls remain compact and easy to scan
- Configuration information becomes more readable in larger windows
- Users can resize the window to see more configuration details without affecting the controller layout

### 6.2 Window Layout

```
┌─────────────────────────────────────────┐
│  Virtual Robot Arm Controller          │
├─────────────────────────────────────────┤
│  [ Joint Mode ] [ Cartesian Mode ]  <-- Tabs
├─────────────────────────────────────────┤
│                                         │
│  Joint 1: [Label] [═══○═══] [  0  ] [R]│
│  Joint 2: [Label] [═══○═══] [  0  ] [R]│
│  ...     (number of joints: configurable)
│  Joint n: [Label] [═══○═══] [  0  ] [R]│
│                                         │
├─────────────────────────────────────────┤
│  Control Panel                          │
│  [Robot: DISABLED] [Home (Cartesian)]   │
├─────────────────────────────────────────┤
│  Configuration Display                  │
│  Frequency: 10 Hz                       │
│  Number of Joints: 6                    │
│  Joint Range: -90° to 90° (rad)         │
│  Topics: /joint_cmd, /cart_cmd, ...     │
└─────────────────────────────────────────┘

Note: Window height automatically adjusts
based on the number of joints configured.
```

### 6.3 UI Components

#### 6.3.1 Axis Control Row
- **Label**: Displays axis name (e.g., "Joint 1", "X", "Roll")
- **Slider**: Horizontal slider for continuous adjustment
  - Range: configured min to max
  - Default position: center (0)
- **Input Field**: Numeric text field
  - Width: 60-80 pixels
  - Shows current slider value
  - Editable: typing updates slider
- **Reset Button**: Small button marked "R" or "Reset"
  - Resets this axis to 0

#### 6.3.2 Control Panel Buttons
- **Enable/Disable Button**
  - Toggle button
  - Enabled state: green background, text "Disable"
  - Disabled state: red background, text "Enable"

- **Home Position Button**
  - Standard push button
  - Resets all current mode axes to home position

#### 6.3.3 Configuration Display
- Read-only text area
- Displays current configuration in readable format
- Updates when configuration file is loaded

### 6.4 User Interactions

1. **Slider Movement**: Updates numeric field in real-time
2. **Numeric Input**: Validates and updates slider position on Enter/focus loss
3. **Reset Button**: Instantly sets axis value to 0
4. **Home Button**: Sets Cartesian axes to configured home position (only enabled in Cartesian mode)
5. **Enable Toggle**: Immediately starts/stops command publishing and enables/disables axis controls
6. **Tab Switch**: Changes between Joint and Cartesian modes, updates home button availability, and automatically disables robot (if configured)

## 7. Technical Implementation Details

### 7.1 Value Scaling

GUI sliders use integer values for smooth operation:
- Internal slider range: -1000 to +1000 (or similar)
- Scale to actual range when publishing
- Display scaled value in numeric field

### 7.2 Timer and Publishing

- QTimer triggers at interval: 1000 / frequency (ms)
- Timer callback:
  1. Read current GUI values
  2. Apply unit conversions
  3. Check enable state
  4. Publish messages if enabled

### 7.3 Error Handling

- **Invalid Config File**: Use default values, log warning
- **Invalid Numeric Input**: Clamp to valid range
- **ROS2 Connection**: Continue GUI operation even if ROS2 fails

### 7.4 Configuration Loading

- Load on startup
- Validate all values
- Apply defaults for missing entries
- Display loaded configuration in UI

## 8. Testing Requirements

### 8.1 Functional Tests
- Mode switching triggers auto-disable (if configured)
- Auto-disable on tab switch can be disabled via config
- Disable stops message publishing
- Home button resets to configured values (Cartesian only)
- Individual reset buttons work correctly
- Unit conversion is accurate

### 8.2 ROS2 Integration Tests
- Messages published at correct frequency
- Message format matches specification
- Enable flags reflect GUI state
- No messages sent when disabled

### 8.3 UI Tests
- Slider and input field stay synchronized
- Invalid input is rejected/clamped
- Configuration display updates correctly
- Tab switching works smoothly

## 9. Future Enhancements (Optional)

- Save current position as new home position
- Load/save multiple configuration profiles
- Real-time feedback from robot (joint states, pose)
- Keyboard shortcuts for common operations
- Command history/recording
