std_msgs/Header header
string session_id
uint64 trajectory_id
uint64 sequence
bool accepted
bool sending
bool offboard_confirmed
bool connected
bool armed
bool landed_known
uint8 landed_state
# PX4 主模式名（HEARTBEAT.custom_mode 的 main_mode，如 auto / offboard）
string mode
# 实际观测到的完整模式，AUTO 下带子模式（如 auto:land / auto:rtl / auto:loiter）。
# 只报主模式无法区分 AUTO Land 与 AUTO Return，恢复逻辑与验收都需要这一项。
string mode_detail
uint8 custom_main_mode
uint8 custom_sub_mode
uint32 restart_epoch
float64 heartbeat_age_s
float64 position_age_s
float64 landed_age_s
bool position_known
geometry_msgs/Point position_ned
geometry_msgs/Vector3 velocity_ned
bool sensors_ready
bool alignment_valid
float64 alignment_yaw_offset_rad
geometry_msgs/Vector3 alignment_translation_m
string[] reasons

# CURRENT_MODE readback; absence must not authorize interruption of Land/Return.
string current_mode_detail
string intended_mode_detail
float64 current_mode_age_s
uint32 frame_reset_epoch

# Authorized local SIH uORB observation; missing outside SIH.
string px4_safety_mode
string px4_failsafe_cause
float64 px4_safety_age_s

# PX4 ATTITUDE observation in NED; absent/stale heading must not create a zero-yaw HOLD.
bool attitude_known
float64 yaw_ned_rad
float64 attitude_age_s

# ODOMETRY.reset_counter observation; zero epoch alone does not prove no reset.
bool frame_reset_known
float64 frame_reset_age_s
