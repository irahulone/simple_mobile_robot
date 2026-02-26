"""Configuration loader for Virtual Robot Arm Controller."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
import math


class ControllerConfig:
    """Configuration class for the virtual controller."""

    # Default values
    DEFAULTS = {
        'topics': {
            'joint_command': '/joint_cmd',
            'cartesian_command': '/cart_cmd',
            'joint_enable': '/joint_en',
            'cartesian_enable': '/cart_en',
            'robot_enable': '/enable',
        },
        'frequency': 10,
        'safety': {
            'disable_on_tab_switch': True,
        },
        'joint': {
            'num_joints': 6,
            'min_angle': -90,
            'max_angle': 90,
            'resolution': 0.1,  # degrees
            'angle_unit': 'rad',
        },
        'cartesian': {
            'position': {
                'min': -0.10,  # meters
                'max': 0.10,   # meters
                'resolution': 0.001,  # meters (minimum step)
                'unit': 'm',
            },
            'orientation': {
                'min': -30,
                'max': 30,
                'resolution': 0.1,  # degrees
                'unit': 'rad',
            },
        },
        'home_position': {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        },
    }

    def __init__(self, config_file=None):
        """
        Initialize configuration.

        Args:
            config_file: Path to YAML config file. If None, uses package config.
        """
        self.config = self._load_config(config_file)
        self._validate_config()

    def _load_config(self, config_file):
        """Load configuration from YAML file with fallback to defaults."""
        if config_file is None:
            try:
                package_dir = get_package_share_directory('virtual_joypad')
                config_file = os.path.join(package_dir, 'config', 'controller_config.yaml')
            except Exception as e:
                print(f"Warning: Could not find package config: {e}")
                print("Using default configuration values")
                return self.DEFAULTS.copy()

        try:
            with open(config_file, 'r') as f:
                loaded_config = yaml.safe_load(f)
                # Merge with defaults
                return self._merge_with_defaults(loaded_config)
        except FileNotFoundError:
            print(f"Warning: Config file not found: {config_file}")
            print("Using default configuration values")
            return self.DEFAULTS.copy()
        except yaml.YAMLError as e:
            print(f"Error parsing YAML config: {e}")
            print("Using default configuration values")
            return self.DEFAULTS.copy()

    def _merge_with_defaults(self, loaded_config):
        """Merge loaded config with defaults for missing values."""
        def deep_merge(default, loaded):
            """Recursively merge dictionaries."""
            if not isinstance(loaded, dict):
                return loaded
            result = default.copy()
            for key, value in loaded.items():
                if key in result and isinstance(result[key], dict):
                    result[key] = deep_merge(result[key], value)
                else:
                    result[key] = value
            return result

        return deep_merge(self.DEFAULTS, loaded_config or {})

    def _validate_config(self):
        """Validate configuration values with comprehensive checks."""
        # Validate required top-level keys
        required_keys = ['topics', 'frequency', 'safety', 'joint', 'cartesian', 'home_position']
        for key in required_keys:
            if key not in self.config:
                raise ValueError(f"Missing required configuration key: '{key}'")

        # Validate frequency
        if not isinstance(self.config['frequency'], (int, float)):
            raise TypeError("Configuration 'frequency' must be a number")
        if self.config['frequency'] <= 0:
            print("Warning: Invalid frequency (must be > 0), using default 10 Hz")
            self.config['frequency'] = 10

        # Validate safety settings
        safety = self.config.get('safety', {})
        if not isinstance(safety.get('disable_on_tab_switch', True), bool):
            print("Warning: 'disable_on_tab_switch' must be boolean, using default True")
            self.config['safety']['disable_on_tab_switch'] = True

        # Validate joint settings
        joint = self.config['joint']

        # Validate num_joints
        if not isinstance(joint.get('num_joints'), int):
            raise TypeError("Configuration 'joint.num_joints' must be an integer")
        if joint['num_joints'] < 1 or joint['num_joints'] > 12:
            print("Warning: Invalid number of joints (must be 1-12), using default 6")
            joint['num_joints'] = 6

        # Validate joint angle range
        if not isinstance(joint.get('min_angle'), (int, float)) or not isinstance(joint.get('max_angle'), (int, float)):
            raise TypeError("Joint angle min/max must be numbers")
        if joint['min_angle'] >= joint['max_angle']:
            print("Warning: Invalid joint angle range (min >= max), using defaults")
            joint['min_angle'] = -90
            joint['max_angle'] = 90

        # Validate joint resolution
        if not isinstance(joint.get('resolution'), (int, float)):
            raise TypeError("Joint resolution must be a number")
        if joint['resolution'] <= 0:
            print("Warning: Invalid joint resolution (must be > 0), using default 0.1 deg")
            joint['resolution'] = 0.1

        # Validate joint angle unit
        valid_angle_units = ['rad', 'deg']
        if joint.get('angle_unit') not in valid_angle_units:
            raise ValueError(f"Invalid joint.angle_unit '{joint.get('angle_unit')}'. Must be one of: {valid_angle_units}")

        # Validate cartesian position
        cart_pos = self.config['cartesian']['position']
        if not isinstance(cart_pos.get('min'), (int, float)) or not isinstance(cart_pos.get('max'), (int, float)):
            raise TypeError("Cartesian position min/max must be numbers")
        if cart_pos['min'] >= cart_pos['max']:
            print("Warning: Invalid cartesian position range (min >= max), using defaults")
            cart_pos['min'] = -0.10  # meters
            cart_pos['max'] = 0.10   # meters

        # Validate resolution
        if not isinstance(cart_pos.get('resolution'), (int, float)):
            raise TypeError("Cartesian position resolution must be a number")
        if cart_pos['resolution'] <= 0:
            print("Warning: Invalid cartesian position resolution (must be > 0), using default 0.001 m")
            cart_pos['resolution'] = 0.001

        # Validate cartesian position unit
        valid_position_units = ['m', 'cm']
        if cart_pos.get('unit') not in valid_position_units:
            raise ValueError(f"Invalid cartesian.position.unit '{cart_pos.get('unit')}'. Must be one of: {valid_position_units}")

        # Validate cartesian orientation
        cart_ori = self.config['cartesian']['orientation']
        if not isinstance(cart_ori.get('min'), (int, float)) or not isinstance(cart_ori.get('max'), (int, float)):
            raise TypeError("Cartesian orientation min/max must be numbers")
        if cart_ori['min'] >= cart_ori['max']:
            print("Warning: Invalid cartesian orientation range (min >= max), using defaults")
            cart_ori['min'] = -30
            cart_ori['max'] = 30

        # Validate cartesian orientation resolution
        if not isinstance(cart_ori.get('resolution'), (int, float)):
            raise TypeError("Cartesian orientation resolution must be a number")
        if cart_ori['resolution'] <= 0:
            print("Warning: Invalid cartesian orientation resolution (must be > 0), using default 0.1 deg")
            cart_ori['resolution'] = 0.1

        # Validate cartesian orientation unit
        if cart_ori.get('unit') not in valid_angle_units:
            raise ValueError(f"Invalid cartesian.orientation.unit '{cart_ori.get('unit')}'. Must be one of: {valid_angle_units}")

        # Validate home position
        home = self.config['home_position']
        required_home_keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for key in required_home_keys:
            if key not in home:
                raise ValueError(f"Missing home_position key: '{key}'")
            if not isinstance(home[key], (int, float)):
                raise TypeError(f"home_position.{key} must be a number")

        # Validate topic names
        topics = self.config['topics']
        required_topic_keys = ['joint_command', 'cartesian_command', 'joint_enable', 'cartesian_enable', 'robot_enable']
        for key in required_topic_keys:
            if key not in topics:
                raise ValueError(f"Missing topic name: '{key}'")
            if not isinstance(topics[key], str) or not topics[key].strip():
                raise ValueError(f"Topic '{key}' must be a non-empty string")

    # Topic getters
    def get_joint_command_topic(self):
        return self.config['topics']['joint_command']

    def get_cartesian_command_topic(self):
        return self.config['topics']['cartesian_command']

    def get_joint_enable_topic(self):
        return self.config['topics']['joint_enable']

    def get_cartesian_enable_topic(self):
        return self.config['topics']['cartesian_enable']

    def get_robot_enable_topic(self):
        return self.config['topics']['robot_enable']

    # Frequency
    def get_frequency(self):
        return self.config['frequency']

    # Safety settings
    def get_disable_on_tab_switch(self):
        """Get whether to disable robot when switching tabs."""
        return self.config['safety']['disable_on_tab_switch']

    # Joint settings
    def get_num_joints(self):
        """Get number of joints."""
        return self.config['joint']['num_joints']

    def get_joint_min_angle(self):
        """Get min angle in degrees (GUI display)."""
        return self.config['joint']['min_angle']

    def get_joint_max_angle(self):
        """Get max angle in degrees (GUI display)."""
        return self.config['joint']['max_angle']

    def get_joint_resolution(self):
        """Get joint angle resolution (minimum step size) in degrees."""
        return self.config['joint']['resolution']

    def get_joint_angle_unit(self):
        """Get joint angle unit for publishing."""
        return self.config['joint']['angle_unit']

    def joint_angle_to_publish_unit(self, degrees):
        """Convert GUI angle (degrees) to publish unit."""
        if self.get_joint_angle_unit() == 'rad':
            return degrees * math.pi / 180.0
        return degrees

    # Cartesian position settings
    def get_cartesian_position_min(self):
        """Get min position in meters."""
        return self.config['cartesian']['position']['min']

    def get_cartesian_position_max(self):
        """Get max position in meters."""
        return self.config['cartesian']['position']['max']

    def get_cartesian_position_resolution(self):
        """Get cartesian position resolution (minimum step size) in meters."""
        return self.config['cartesian']['position']['resolution']

    def get_cartesian_position_unit(self):
        """Get cartesian position unit for publishing."""
        return self.config['cartesian']['position']['unit']

    def cartesian_position_to_publish_unit(self, meters):
        """Convert position from meters (GUI/config unit) to publish unit.

        Args:
            meters: Position value in meters

        Returns:
            Position value in the configured publish unit (m or cm)
        """
        if self.get_cartesian_position_unit() == 'cm':
            return meters * 100.0
        else:  # unit == 'm'
            return meters

    # Cartesian orientation settings
    def get_cartesian_orientation_min(self):
        """Get min orientation in degrees (GUI display)."""
        return self.config['cartesian']['orientation']['min']

    def get_cartesian_orientation_max(self):
        """Get max orientation in degrees (GUI display)."""
        return self.config['cartesian']['orientation']['max']

    def get_cartesian_orientation_resolution(self):
        """Get cartesian orientation resolution (minimum step size) in degrees."""
        return self.config['cartesian']['orientation']['resolution']

    def get_cartesian_orientation_unit(self):
        """Get cartesian orientation unit for publishing."""
        return self.config['cartesian']['orientation']['unit']

    def cartesian_orientation_to_publish_unit(self, degrees):
        """Convert GUI orientation (degrees) to publish unit."""
        if self.get_cartesian_orientation_unit() == 'rad':
            return degrees * math.pi / 180.0
        return degrees

    # Home position
    def get_home_position(self):
        """Get home position as dictionary."""
        return self.config['home_position'].copy()

    # Configuration display string
    def get_config_display_string(self):
        """Get formatted configuration string for GUI display."""
        lines = []
        lines.append("=== Configuration ===")
        lines.append(f"Frequency: {self.get_frequency()} Hz")
        lines.append(f"Disable on Tab Switch: {'Yes' if self.get_disable_on_tab_switch() else 'No'}")
        lines.append("")

        lines.append("Topics:")
        lines.append(f"  Joint Command: {self.get_joint_command_topic()}")
        lines.append(f"  Cartesian Command: {self.get_cartesian_command_topic()}")
        lines.append(f"  Joint Enable: {self.get_joint_enable_topic()}")
        lines.append(f"  Cartesian Enable: {self.get_cartesian_enable_topic()}")
        lines.append(f"  Robot Enable: {self.get_robot_enable_topic()}")
        lines.append("")

        lines.append("Joint Mode:")
        lines.append(f"  Number of Joints: {self.get_num_joints()}")
        lines.append(f"  Range: {self.get_joint_min_angle()}° to {self.get_joint_max_angle()}°")
        lines.append(f"  Resolution: {self.get_joint_resolution():.1f}°")
        lines.append(f"  Publish Unit: {self.get_joint_angle_unit()}")
        lines.append("")

        lines.append("Cartesian Mode:")
        lines.append(f"  Position Range: {self.get_cartesian_position_min():.3f} to {self.get_cartesian_position_max():.3f} m")
        lines.append(f"  Position Resolution: {self.get_cartesian_position_resolution():.3f} m")
        lines.append(f"  Position Publish Unit: {self.get_cartesian_position_unit()}")
        lines.append(f"  Orientation Range: {self.get_cartesian_orientation_min()}° to {self.get_cartesian_orientation_max()}°")
        lines.append(f"  Orientation Resolution: {self.get_cartesian_orientation_resolution():.1f}°")
        lines.append(f"  Orientation Unit: {self.get_cartesian_orientation_unit()}")
        lines.append("")

        home = self.get_home_position()
        lines.append("Home Position:")
        lines.append(f"  X: {home['x']}, Y: {home['y']}, Z: {home['z']}")
        lines.append(f"  Roll: {home['roll']}, Pitch: {home['pitch']}, Yaw: {home['yaw']}")

        return "\n".join(lines)
