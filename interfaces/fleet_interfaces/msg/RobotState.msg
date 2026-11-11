string STATE_IDLE="IDLE"
string STATE_MOVING="MOVING"
string STATE_WAITING="WAITING"
string STATE_DOCKING="DOCKING"
string STATE_DOCKED="DOCKED"
string STATE_ERROR="ERROR"
string STATE_BLOCKED="BLOCKED"
string STATE_LOCALIZING="LOCALIZING"
string STATE_RECOVERING="RECOVERING"

string robot_id
geometry_msgs/Pose2D pose
float32 battery
string state
string current_goal
builtin_interfaces/Time stamp

# 막힘 물체 보고(제안, 계약 0.10-draft, 팀 합의 전). BLOCKED와 RECOVERING일 때 로봇이 본
# 경로 위 물체. obstacle_seen이 false면 아래 값은 쓰지 않는다(옛 로봇은 늘 false)
bool obstacle_seen
float32 obstacle_x  # 물체 중심, 지도 좌표 m
float32 obstacle_y
float32 obstacle_radius  # m, 0이면 관제가 기본 반지름을 쓴다
