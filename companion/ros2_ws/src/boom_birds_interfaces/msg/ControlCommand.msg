uint8 EXECUTE=1
uint8 CANCEL=2
uint8 HOLD=3
std_msgs/Header header
string session_id
string planner_session_id
string executor_session_id
uint64 trajectory_id
uint64 sequence
builtin_interfaces/Duration valid_for
uint8 command_type
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 yaw
float64 yaw_rate

# VIO 正常降落 HOLD；接地降推力必须使用同一份新鲜、已授权命令。
bool landing
float64 ground_z_world_m
