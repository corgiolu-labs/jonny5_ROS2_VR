# Parsed telemetry from the STM32 SPI data plane.

builtin_interfaces/Time stamp
uint32 packet_index
uint16 sequence_counter
uint8 frame_type
bool header_ok
bool telemetry_fresh
bool imu_valid
uint32 imu_sample_counter
geometry_msgs/Quaternion imu_orientation
float32[6] servo_deg
uint16 rt_loop_period_us
uint16 rt_step_us
uint8 raw_mode
uint16 raw_heartbeat
uint16 diag_mask
