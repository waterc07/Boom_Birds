# ******************************************************************************
# @file           : PlannerStatus.msg
# @brief          : 报告规划器当前会话、地图就绪、几何故障和目标状态。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

std_msgs/Header header
string session_id
string producer_session_id
bool map_ready
bool geometry_fault
bool goal_active
