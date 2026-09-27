# Parsed telemetry from the STM32 SPI data plane.

builtin_interfaces/Time stamp
uint32 packet_index
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

# --- SPI protocol v2 (all zero/false when the link runs protocol v1) ---
uint8 protocol_version
# STM32 uptime (ms) when the reply was built; use it to align firmware time.
uint32 stm_time_ms
# Firmware reply counter: equal values on consecutive frames = repeated reply.
uint16 stm_tx_seq
# Firmware state machine: 0 SAFE, 1 IDLE, 2 STOPPED.
uint8 fsm_state
bool estop_active
bool setpose_active
bool joint_stream_active
uint16 rt_overruns
# Pi->STM32 frames lost (sequence gaps) and CRC failures seen by the firmware.
uint16 rx_seq_gaps
uint16 crc_errors_stm
# STM32->Pi frames rejected by the Pi for bad CRC, and repeated replies.
uint32 crc_errors_pi
uint32 repeated_replies
