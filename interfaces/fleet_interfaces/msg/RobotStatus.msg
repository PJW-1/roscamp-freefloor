# One robot as the FMS sees it, for the monitoring UI (SR-17).
string FAULT_NONE="NONE"
string FAULT_ERROR="ERROR"
string FAULT_COMM_LOST="COMM_LOST"
string FAULT_OUT_OF_SERVICE="OUT_OF_SERVICE"
string FAULT_RECOVERING="RECOVERING"

# fsm_state, battery, x, y and yaw are its last report, however old that is. Only fault says a
# robot has gone silent, and only while it has no other fault.
string robot_id
string fsm_state        # what the pinky reported, as it reported it. Empty before its first report
string task_type        # Assignment.TYPE_*. Empty when it is off the floor
string order_id
string fault            # FAULT_*
string fault_reason     # why, in a word or the operator's note. Empty when fault is NONE
string wait_zone        # the zone it queues for. Empty when it is not waiting
string[] wait_robots    # who has to get out of the way first
float32 battery
float64 x
float64 y
float64 yaw
