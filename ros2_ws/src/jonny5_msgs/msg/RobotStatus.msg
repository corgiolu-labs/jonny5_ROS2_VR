# High-level JONNY5 robot state suitable for diagnostics and UI.

builtin_interfaces/Time stamp
string state
bool movement_allowed
bool deadman_active
bool input_active
bool spi_online
bool stm32_online
bool imu_online
# Protocol v2 only: hardware E-STOP latched on the STM32.
bool estop_active
string detail
