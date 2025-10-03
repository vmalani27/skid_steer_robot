# RobotStatus.msg
# Overall status of the robot

std_msgs/Header header

# Robot state
string robot_state  # e.g., "IDLE", "MOVING", "OBSTACLE_DETECTED", "ERROR"

# Battery status (if available)
float32 battery_voltage
float32 battery_percentage

# Motor status
bool left_motor_enabled
bool right_motor_enabled
float32 left_motor_current
float32 right_motor_current

# Sensor status
bool sensors_active
int32 active_sensor_count

# Navigation status
bool navigation_active
string current_behavior  # e.g., "WANDER", "OBSTACLE_AVOIDANCE", "MANUAL"

# Error messages
string[] error_messages
string[] warning_messages