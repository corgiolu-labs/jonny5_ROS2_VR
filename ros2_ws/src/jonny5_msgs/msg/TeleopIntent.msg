# Normalized JONNY5 VR teleoperation intent.

builtin_interfaces/Time stamp

uint8 MODE_IDLE=0
uint8 MODE_RELATIVE_MOVE=1
uint8 MODE_MANUAL=2
uint8 MODE_HEAD=3
uint8 MODE_HYBRID=4
uint8 MODE_ASSIST=5

uint8 mode
int16 joy_x
int16 joy_y
int16 pitch
int16 yaw
uint8 intensity
bool grip
uint16 heartbeat
uint8 priority
uint16 safe_mask
geometry_msgs/Quaternion headset_orientation
uint16 buttons_left
uint16 buttons_right

bool mode5_arm_valid
bool mode5_grip_active
bool mode5_hold_active
uint16 mode5_target_id
float32 mode5_base_deg
float32 mode5_shoulder_deg
float32 mode5_elbow_deg
