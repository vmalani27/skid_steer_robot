# UltrasonicSensorData.msg
# Message containing data from all ultrasonic sensors

std_msgs/Header header

# Sensor readings in meters
float32 front_distance
float32 rear_distance
float32 left_distance
float32 right_distance

# Sensor status (true if reading is valid)
bool front_valid
bool rear_valid
bool left_valid
bool right_valid

# Obstacle detection flags
bool obstacle_front
bool obstacle_rear
bool obstacle_left
bool obstacle_right