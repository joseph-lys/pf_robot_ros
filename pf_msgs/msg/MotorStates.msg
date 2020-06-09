uint8 ERROR_INPUT_VOLTAGE = 1
uint8 ERROR_ANGLE_LIMIT = 2
uint8 ERROR_OVERHEATING = 4
uint8 ERROR_RANGE = 8
uint8 ERROR_CHECKSUM = 16
uint8 ERROR_OVERLOAD = 32
uint8 ERROR_INSTRUCTION = 64
uint8 ERROR_COMMUNICATION = 128

Header header
bool comm_ok
bool torque_enabled
bool control_enabled
string[] joint_names
uint8[] motor_ids
uint8[] motor_states

