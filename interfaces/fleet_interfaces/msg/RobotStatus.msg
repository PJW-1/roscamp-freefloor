# FMS가 보는 로봇 한 대의 상태. 모니터링 UI용(SR-17).
string FAULT_NONE="NONE"
string FAULT_ERROR="ERROR"
string FAULT_COMM_LOST="COMM_LOST"
string FAULT_OUT_OF_SERVICE="OUT_OF_SERVICE"
string FAULT_RECOVERING="RECOVERING"

# fsm_state, battery, x, y, yaw는 마지막 보고값이며 아무리 오래됐어도 그대로다.
# 로봇이 조용해졌는지는 fault로만 알 수 있고, 다른 fault가 없을 때만 그렇다.
string robot_id
string fsm_state        # 핑키가 보고한 값 그대로. 첫 보고 전에는 빈 문자열
string task_type        # Assignment.TYPE_*. 현장에서 빠졌으면 빈 문자열
string order_id
string fault            # FAULT_*
string fault_reason     # 이유(한 단어 또는 운영자 메모). fault가 NONE이면 빈 문자열
string wait_zone        # 대기 중인 구역. 대기 중이 아니면 빈 문자열
string[] wait_robots    # 먼저 비켜 줘야 하는 로봇
float32 battery
float64 x
float64 y
float64 yaw
