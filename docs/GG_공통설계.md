# GG 공통 설계 (코드와 문서가 함께 따르는 기준, 초안)

기준은 Confluence에 있는 것이 최종입니다(2026-11-04 확정).
- 기준 UR: Confluence "주제1 : GG(Give Gift)" 페이지(425992, UR-01~14). 배터리·홈 복귀는 UR-10, 관리자 실시간 확인은 UR-11, 오류 알림은 UR-12, 충돌·간섭 방지는 UR-13, 이력 확인은 UR-14. 2785281(UR-01~15)은 더 이상 쓰지 않습니다.
- 기준 SR: Confluence "System Requirements" 페이지(819201, SR-01~38, UR 칸 포함). UR 칸의 "-"는 최종 UR에 대응 항목이 없다는 뜻입니다. "Copy of System Requirements"는 작업본이며 기준이 아닙니다.
- 계층(2026-11-05, v17 FMS 통합 판, 팀 합의 전): 사용자(화면) → 서버(central, FMS) → 로봇(핑키, 로봇팔). 위가 아래를 부르고 아래는 상태와 결과만 올린다. FMS는 프로세스 하나(`fms_node`)이고, 그 안에 주문 진행을 맡는 `orchestrator_core`와 교통 제어를 맡는 `traffic_core`가 파이썬 호출로 연결된다. 4·5·5-1절은 이 구조로 고쳤다. 10·11·13·14절은 아직 합치기 전의 설명이 남아 있고 코드 분리 뒤에 고친다. 이 문서와 Confluence가 다른 것으로 확인한 항목은 12절에 모았고, 어느 쪽으로 맞출지는 아직 정하지 않았다.
- 최종 갱신: 2026-11-05. orchestrator와 fleet을 FMS 한 프로세스로 합치고, 둘 사이의 ROS 액션(`/fleet/dispatch`)과 서비스(`/fleet/release`), 중계 토픽(`/fleet/events`, `/orchestrator/*`)을 없앴다(4절). 작업본은 저장소 `docs/GG_공통설계.md`이고 Drive 원본(2026-11-05 00:51 수정, 40 KB)은 고치지 않았다.

## 1. 확정 시나리오 (사용자가 확정, 바꾸지 말 것)
주문이 들어오면 핑키가 대기 구간에서 출발한다. 공유 작업공간의 로봇팔 2대가 핑키 뒤 트레이의 바구니에 주문 상품을 담는다.
핑키가 출고 구역으로 간다. 로봇팔 1대가 바구니째 꺼내고 빈 바구니를 트레이에 넣는다. 핑키는 대기 구간으로 돌아가거나, 주문이 밀려 있으면 바로 작업공간으로 간다.

## 2. 가정 (팀 미확정, 문서에 "가정"으로 표시)
- 작업공간 위치는 미정이라 지도는 임시 배치다.
- 작업공간 도크에는 핑키 1대만 들어간다. 나머지는 대기열에서 기다린다.
- 팔 2대는 상품 영역 1곳을 함께 쓴다. workcell이 두 팔을 하나의 플래닝 그룹으로 묶어 궤적을 함께 계획하므로 둘이 동시에 움직이고, 한 번에 1대만 들어가는 잠금 방식은 쓰지 않는다(2026-11-04 결정). 어느 팔이 어느 품목을 맡는지는 workcell이 정한다. 팔별로 상품을 나눠 두는 배치는 처리량 비교용 베이스라인으로 남긴다.
- 주문 1건 = 바구니 1개. 바구니당 최대 4개(`basket_capacity`, 실제 바구니로 측정 필요).
- 검수 없음. 재고는 중앙이 관리하고 관리자가 수동 정정한다.
- 모든 핑키는 빈 바구니를 싣고 시작한다.
- 결품이면 담긴 것만 출고(`ship_partial`), 하나도 못 담으면 보류(HELD)하고 빈 바구니로 다음 일을 한다.

## 3. 지도 (`pc/fleet_core/fleet_core/data/site_map.yaml`, 확정된 초안)
12×7칸, 칸 0.33 m. 스테이션 `WORK`(도크 (5,3), 대기열 (4,3),(3,3), 서버 `workcell`, 팔 arm1·arm2, 동작 `fill`)와 `OUTBOUND`(도크 (9,3), 대기열 (9,4),(9,5), 서버 `arm3`, 동작 `swap`).
일방통행 한 바퀴: WORK → to_outbound(flow out, 9칸) → OUTBOUND → to_work(flow ret, 17칸) → WORK. 대기 구간은 복귀 차선의 (2,3)에서 갈라져 waiting_loop(flow wait, P-1~P-6)를 돌아 (3,3)에서 합류한다. 교차는 없고 합류만 있다.
구역 16개: Z-WORK, Z-O, Q-OUT-2, Q-OUT-1, Z-OUT, Z-R1, Z-R2, Z-R3, Q-WORK-2, Q-WORK-1, P-1~P-6.
로봇 3대 전수 검사: `(149238, 0, 0)`(상태, 교착, 기아). 차선이 한 바퀴뿐이라 대기 칸 밖에서 멈춘 로봇 1대는 나머지를 모두 막는다. 이것은 설계상 약점으로 문서에 밝힌다.
이 배치는 임시다(2절). 2026-11-05 맵 아키텍처 논의에서 정할 항목은 `09_맵_아키텍처_결정_항목.txt`에 있다.

## 4. 인터페이스 (`interfaces/fleet_interfaces`, FMS 통합 구조 기준)
계층: 사용자 → central → FMS → 핑키 / workcell(arm1·arm2) / arm3. central은 로봇을 직접 움직이지 않고, 로봇은 central을 모른다. 핑키 파트는 팔을 모르고 팔 파트는 핑키를 모른다.

**FMS 안의 연결은 ROS가 아니라 파이썬 호출이다.** `orchestrator_core`와 `traffic_core`는 한 프로세스(`fms_node`)의 두 모듈이므로 둘 사이에 액션, 서비스, 토픽을 두지 않는다. 합치기 전 판에 있던 `/fleet/dispatch` 액션, `/fleet/release` 서비스, `/fleet/events`, `/orchestrator/assignments`, `/orchestrator/zones`, `/orchestrator/{arm}/*`, `/orchestrator/confirm_clear`는 만들지 않는다. `Dispatch.action`과 `Release.srv`도 만들지 않는다.

FMS 내부 호출 (파이썬, `fleet_core` 패키지):
- `on_dispatch(goal_id, station_id, robot_id="")`: orchestrator → traffic. 로봇을 스테이션으로 보내 달라는 요청. `robot_id`가 비면 traffic이 고른다.
- `on_dispatch_cancel(goal_id)`: orchestrator → traffic. 아직 끝나지 않은 요청을 취소한다.
- `on_release(robot)`: orchestrator → traffic. 스테이션에서 할 일이 끝난 로봇을 놓아준다.
- `DispatchFeedback(goal_id, robot_id, phase)`, `DispatchResult(goal_id, success, robot_id, reason, ex, ey, eyaw)`: traffic → orchestrator. 반대 방향 보고. `phase`는 CALLED, EN_ROUTE, QUEUED, DOCKING, NO_ROBOT. 결과가 success면 그 로봇은 도크에 서서(AT_STATION) `on_dispatch(robot_id 지정)`나 `on_release`를 기다린다.

두 모듈 사이는 위가 전부이고, `traffic_core`는 `orchestrator_core`를 import하지 않으며 주문과 바구니 타입을 쓰지 않는다(경계 시험 `test_fms_boundary.py`). 다만 다음 세 가지는 이 호출로 표현할 수 없어 둘을 묶는 `FmsCore`가 따로 전달한다. 지시서의 "네 호출"에 없는 항목이므로 팀에 알린다.
- `on_run_mode(mode)`: central의 STOP/PAUSE/RUN을 두 모듈에 각각 넘긴다. 주문과 상관없이 달리던 로봇도 멈춰야 하므로 dispatch 취소로는 안 된다.
- `on_robot_removed(robot)`: `confirm_clear`가 받아들여지면 orchestrator에 알린다. 담기나 교환 중이면 돌려줄 dispatch가 이미 끝나 있기 때문이다.
- `on_robot_fault(robot, reason)`: 팔 재시도가 소진되어 orchestrator가 ERROR로 판단하면 traffic에 알린다. 통신 두절 판단과 호출 후보, "남은 로봇 없음" 판단이 이 상태를 본다.

바깥 인터페이스 (`contract.yaml`과 같고 이번 구조 변경으로 이름, 타입, QoS가 바뀌지 않는다):
- `/central/submit_order` SubmitOrder 서비스: UI → central. 재고 기록에 없거나 용량 초과면 거절
- `/central/set_run_mode` 서비스: UI → central (RUN / PAUSE / STOP)
- `/central/orders` OrderBoard 토픽(run_mode, 최종 보고가 안 된 주문을 오래된 순서로): central → FMS
- `/central/report_order` ReportOrder 서비스(SHIPPED, PARTIAL, HELD, placed, missing, detail): FMS → central
- `/central/events` FleetEvent 토픽: FMS와 central이 발행하고 source로 구분한다
- `/fleet/assignments` AssignmentArray 토픽(항목마다 robot_id, order_id, station_id, task_type: TO_WORK/FILL/TO_OUTBOUND/SWAP/WAIT): FMS → central
- `/fleet/zones` ZoneStateArray 토픽: FMS → central
- `/fleet/confirm_clear` ConfirmClear 서비스: central(관리자) → FMS
- `/{pinky}/state` 토픽, `/{pinky}/move_to`·`/{pinky}/dock` 액션: FMS ↔ 핑키
- `/workcell/fill_basket` FillBasket 액션(items → placed, missing, success, reason): FMS → workcell. 팔 2대 배분과 조율은 workcell 내부 책임
- `/arm3/swap_basket` SwapBasket 액션(take_full, put_empty → took_full, put_empty_done): FMS → arm3
- `/{arm}/state` ArmState 토픽, `/{arm}/vision/items` ItemArray 토픽(arm1, arm2): 팔 파트가 발행한다. FMS의 판단에는 쓰지 않는다
- `/{pinky}/navigate_to_pose` 액션(Nav2): 핑키 안에서 pinky_fsm → Nav2. contract.yaml에 `real_only`로 있고, FMS는 부르지 않는다
- `/cmd_vel`(Nav2, pinky_dock → pinky_bringup), `/scan`, `/odom`(pinky_bringup → Nav2): 핑키 안의 연결이다. 제조사 패키지와 Nav2의 일반적인 이름이고 contract.yaml에는 없다. 3대를 namespace로 나누는 구성은 시험 전이다

새로 제안하는 바깥 인터페이스 (팀 합의 전. 코드와 `contract.yaml`에는 들어 있다):
- `/fleet/robots` RobotStatusArray 토픽(LATCHED, 1 Hz): FMS → central. 로봇마다 robot_id, fsm_state(핑키 보고 그대로), task_type, order_id, fault(NONE, ERROR, COMM_LOST, OUT_OF_SERVICE, RECOVERING), fault_reason, wait_zone, wait_robots, battery, x, y, yaw. `/fleet/assignments`와 `/fleet/zones`는 호환을 위해 유지한다. 위치, 배터리, fsm_state는 마지막 보고 값이다
- `/fms/heartbeat` std_msgs/Header 토픽(STATE QoS, 2 Hz, `heartbeat_period_s` 0.5 s): FMS → 핑키. **핑키 FSM 계약(FSM 담당과 확인 필요):**
  - 이동 또는 도킹 목표를 실행 중일 때, 마지막 heartbeat 또는 목표 수락에서 `heartbeat_timeout_s`(제안 1.5 s, heartbeat 3번)가 지나면 그 목표를 취소하고 정지한다. 목표 수락도 FMS가 살아 있다는 증거로 친다. 새 FMS의 첫 목표가 첫 heartbeat보다 먼저 올 수 있기 때문이다.
  - 할 일이 없는 로봇은 heartbeat가 없어도 아무것도 하지 않고 상태 보고를 계속한다. 멈춘 뒤 온 새 목표는 받는다.
  - 시간은 벽시계로 잰다. 프로세스를 감시하는 값이기 때문이다.
  - 통신 두절도 같다. FMS가 살아 있어도 heartbeat를 못 받으면 멈춘다.
  - **FMS 쪽 약속:** 시작한 뒤 `heartbeat_timeout_s + heartbeat_period_s`(2 s) 동안 heartbeat를 내지 않고 상태 보고도 받지 않는다. 이전 FMS의 목표로 달리던 로봇은 보통 그 사이에 멈춘다. 늦게 수락된 목표나 감속 중인 로봇은 아직 움직일 수 있으므로 침묵만 믿지 않는다. 그 뒤 로봇의 `move_to`, `dock` 서버를 찾았고, 전원이 위치를 보고했고, `MOVING`이나 `DOCKING`이 아니어야 지도에 올리고 움직이기 시작한다.
  - **이 계약이 보장하지 않는 것:** 마지막 heartbeat 뒤 도착한 목표는 수락 시각부터 다시 `heartbeat_timeout_s`를 센다. heartbeat는 best effort라 3번 연달아 잃으면 FMS가 살아 있어도 멈춘다. 멈춘 로봇의 감속 거리는 timeout에 들어 있지 않다.

run_mode는 FMS가 OrderBoard로 받는다. 삭제된 것: TaskBoard, StationCount, BoxArray, ArmTransfer, ReportDone, `/{arm}/load`, `/{arm}/unload`, `/central/tasks`, `/central/report_done`.

## 5. 교통 제어 규칙 (`traffic_core`, 핑키만)
유지: 구역 잠금(현재+다음 1개), 구역별 FIFO, 위치 3회 확인 후 해제, 교착 감지, 통신 두절/복구, 이동 재시도, 시작 시 전원 보고 대기.
교착 처리: 대기 고리가 있는 동안 새 호출과 새 이동 단계를 내지 않는다. 이미 보낸 이동 목표는 그대로 두고, 아래의 포기 대상 로봇만 목표를 취소한다. 고리가 생기고 `deadlock_grace_s`가 지나면 우선순위가 가장 낮은 로봇 1대에서 아직 들어가지 않은 구역을 포기 대상으로 정하고, 대상이 있으면 이동 목표를 취소한다(`DEADLOCK_RETREAT`). 실제 해제는 `recover_check_s` 뒤에 위치 보고로 벗어난 것이 확인된 구역만 한다. 들어간 구역에서 물러나는 동작은 없다. 포기할 구역이 없으면 바로 `DEADLOCK_UNRESOLVED`로 알린다. 재확인 때 구역이 하나라도 해제되면 이 이벤트는 나지 않는다. 아무것도 해제하지 못했어도 대상 중 하나라도 해제 여유 밖에 있으면 재확인을 반복하며, 이 대기에는 시간 제한이 없다. 아무것도 해제하지 못했고 모든 대상의 해제 여유 안에 들어가 있으면 `DEADLOCK_UNRESOLVED`로 알린다(7절, SR-13과 다른 점은 12절).
우선순위 입력: 스테이션에 서 있는지(AT_STATION)와 출고 구역으로 가는 요청을 받았는지만 본다. 주문과 바구니는 보지 않는다.
orchestrator로 옮긴 것(5-1절): 주문 묶기, FILLING/SWAPPING, 팔 재시도·시간 초과, 바구니 상태, ReportOrderCmd.
삭제: 양쪽 물량 비율 배정(Rebalancer의 2:1, flip), 상자 개수, 비전 개수로 적재량 추정(`_moved_by_vision`).

호출(dispatch 처리):
- 로봇 이름 없는 `on_dispatch(goal, WORK)`는 열린 호출로 쌓인다. traffic은 매 주기 열린 호출 수만큼 로봇을 고른다. 후보는 할당이 없는 사용 가능 로봇(AT_STATION이면 `on_release`를 받은 로봇)이고, 일방통행 경로 길이가 가장 짧은 로봇을 고른다(`score_distance_weight`, `score_same_call`). 대기 칸에서는 앞 로봇을 추월할 수 없으므로 앞 로봇부터 호출한다. 고장 난 로봇 뒤의 대기 로봇은 호출하지 않는다. 호출은 주기마다 다시 계산하므로 더 가까운 로봇이 생기면 호출이 옮겨 갈 수 있다. 사용 가능 로봇이 하나도 없으면 호출을 연 채 기다리고 `DispatchFeedback(phase=NO_ROBOT)`을 보낸다(멈춤 한 번에 한 번). orchestrator가 이것을 받아 `NO_ROBOT_AVAILABLE`(WARN)을 낸다.
- 호출이 줄었는데(`on_dispatch_cancel`) 그 로봇이 `commit_zone`(Q-WORK-2) 전이면 대기 구간으로 돌린다. 지났으면 끝까지 가서 도킹 없이 한 바퀴 돌아 대기 구간으로 간다.
- 도킹을 마치면 `DispatchResult`(success, robot_id, 도킹 오차)를 돌려주고 로봇은 AT_STATION이 된다. 다음 `on_dispatch`(robot_id 지정)나 `on_release`가 올 때까지 도크를 비우지 않는다.
- `on_dispatch(goal, OUTBOUND, robot_id=X)`는 지정한 로봇을 보낸다. `on_release(X)`를 받은 로봇은 다음 주기의 호출 후보가 된다. 뽑히면 바로 작업공간으로 가고(작업 도크에 서 있었으면 그 자리에서 결과를 돌려준다), 안 뽑히면 대기 구간으로 간다. 합치기 전 판의 "밀린 주문이면 orchestrator가 그 로봇을 지정해 다시 부른다"는 쓰지 않는다. 대기 칸에 더 가까운 로봇이 있어도 먼 로봇을 부르게 되어 지금 동작과 달라지기 때문이다.

임무 단계: IDLE, PARKED, TO_PARK, TO_WORK, TO_OUTBOUND, DOCKING, AT_STATION (고장 표시: ERROR, OUT_OF_SERVICE, COMM_LOST, RECOVERING). 바구니 상태와 주문은 traffic이 갖지 않는다.

STOP: 이동 목표를 취소한다. 열린 dispatch는 그대로 두고 RUN이면 이어서 간다. PAUSE: 진행 중인 이동은 끝내고 새 이동을 내지 않는다.

confirm_clear(관리자가 로봇을 치웠다): 구역 해제, OUT_OF_SERVICE, 이동 목표 취소. 그 로봇을 지정한 dispatch가 진행 중이면 실패(reason=ROBOT_REMOVED)로 돌려준다. 주문을 되돌리는 것은 orchestrator가 한다(`FmsCore`가 `on_robot_removed`를 부른다).

이벤트 코드(traffic): `DOCKED`, `GOAL_RETRY`, `GOAL_FAILED`, `COMM_LOST`, `COMM_RESTORED`, `ZONE_WAIT_LONG`, `ZONES_CLEARED`, `DEADLOCK_*`, `PLACEMENT_CONFLICT`, `RECOVERY_CONFLICT`, `ROBOT_NOT_REPORTING`.

traffic_core 공개 API:
```python
TrafficCore(site, params, robots, clock, next_goal_id=None)
on_robot_state(robot, x, y, yaw, state); on_move_result(robot, goal_id, success, reason)
on_goal_rejected(robot, goal_id); on_dock_result(robot, goal_id, success, ex, ey, eyaw)
on_dispatch(goal_id, station_id, robot_id=""); on_dispatch_cancel(goal_id); on_release(robot)
on_run_mode(mode); on_robot_fault(robot, reason)
confirm_clear(robot, zone_ids, note="") ; tick() -> list ; zones_view() ; robot_view(robot)
```
명령: MoveCmd, DockCmd, CancelCmd, DispatchFeedback, DispatchResult, EventOut. 뷰: ZoneView.

## 5-1. 주문 진행 규칙 (`orchestrator_core`, FleetCore에서 옮긴 것)
주문 배정:
- **주문은 핑키가 WORK 도크에 도킹을 마친 순간 묶는다.** WORK 요청의 결과가 success로 오면 그 robot_id에 대기 주문 맨 앞을 묶고 `ORDER_BOUND`를 낸다. 그 전에는 "호출"만 한다. 도크 도착 순서가 곧 작업 순서이므로, 이렇게 해야 주문이 등록 순서대로 처리되고 도중에 고장 난 로봇이 주문을 쥐고 있지 않는다.
- 열어 두는 WORK 요청 수 = 아직 묶이지 않은 주문 수. 묶이지 않은 주문이 줄면 남는 요청을 `on_dispatch_cancel`로 취소한다(대기 구간 복귀는 traffic의 commit_zone 규칙).

주문 상태: PENDING, BOUND, FILLING, TO_OUTBOUND, SWAPPING, 종료(SHIPPED, PARTIAL, HELD). 바구니 상태 `basket[robot]`: EMPTY, FULL, NONE. 모든 핑키는 EMPTY로 시작한다.

FILLING:
1. 남은 품목 = 주문 품목 − 담은 것 − 최종 결품. 남은 것이 있으면 FillCmd를 workcell에 보낸다.
2. 결과 success=true: placed, missing을 누적하고 끝낸다. success=false: placed만 누적하고 재시도(`move_retry_limit` 초과 시 ERROR, 주문은 묶인 채 유지). reason=CANCELED는 시도 횟수에 넣지 않는다.
3. 담은 것이 1개 이상이면 basket=FULL, `on_dispatch(OUTBOUND, robot_id=X)`로 출고 구역으로 보낸다. 0개면 HELD 보고 후 주문을 풀고 basket=EMPTY로 `on_release(X)`. 주문이 `basket_capacity`를 넘으면 도킹 후 팔을 부르지 않고 HELD(detail에 사유). `ship_partial`이 false면 일부만 담겨도 FULL로 보내지 않고 HELD한다(담긴 품목은 detail에 적고 관리자 확인이 필요하므로 ERROR 처리).

SWAPPING(OUTBOUND 요청의 결과를 받은 뒤):
1. SwapCmd(take_full = basket==FULL, put_empty=true)를 arm3에 보낸다.
2. took_full이면 basket=NONE으로 바꾸고 **바로** SHIPPED(결품이 있었으면 PARTIAL) 보고 후 주문을 푼다. put_empty_done이면 basket=EMPTY.
3. 남은 단계만 재시도. 한도 초과면 ERROR(`SWAP_FAILED`). basket=EMPTY가 되면 `on_release(X)`. 그다음 어디로 갈지는 traffic이 정한다(5절의 호출).

팔 실패나 거절 뒤에는 `arm_retry_wait_s`(추정 2초)를 기다렸다가 다시 보낸다. 취소한 작업이 있으면 그 결과가 오거나 `arm_cancel_wait_s`가 지날 때까지 같은 서버에 다음 작업을 보내지 않는다.

팔 시간 초과: `fill_timeout_s`/`swap_timeout_s` 초과 시 취소를 보내고 `arm_cancel_wait_s`(추정 5초) 동안 결과를 기다린다. STOP으로 취소할 때도 같은 시간을 기다린다. 결과가 오면 그대로 처리, 안 오면 ERROR(`ARM_NO_RESULT`, 바구니 내용을 알 수 없으니 관리자 확인).

STOP: 팔 작업을 취소한다. RUN으로 돌아오면 남은 일부터 다시 한다. PAUSE: 진행 중인 것은 끝내고 새 명령을 내지 않는다.

로봇 제거(`on_robot_removed`): 묶인 주문이 아직 출고 전이면 대기 주문의 맨 앞(등록 순서 유지)으로 되돌리고 `ORDER_REQUEUED`를 낸다. detail에 담겨 있던 품목을 적는다(되돌리거나 재고를 고치는 것은 관리자 몫). 진행 중이던 팔 작업으로 담긴 품목은 detail에 들어가지 않으므로, 실제 바구니는 관리자가 직접 확인한다. 그 로봇의 팔 작업이 진행 중이면 취소를 보내고 위의 취소 대기 규칙을 따른다. 취소한 작업의 결과가 늦게 와도 주문과 바구니에 반영하지 않는다. 같은 제거가 두 번 와도 주문은 한 번만 되돌린다.

이벤트 코드(orchestrator, 심판과 맞추는 약속): `ORDER_BOUND`(detail=주문 ID, source=로봇), `ORDER_REQUEUED`(detail이 주문 ID로 시작), `FILL_RETRY`, `SWAP_RETRY`, `FILL_FAILED`, `SWAP_FAILED`, `ARM_NO_RESULT`, `NO_ROBOT_AVAILABLE`.
central이 내는 코드: `ORDER_ACCEPTED`, `ORDER_REJECTED`, `ORDER_SHIPPED`, `ORDER_PARTIAL`, `ORDER_HELD`(detail이 주문 ID로 시작), `OUT_OF_STOCK`(재고 기록도 0), `STOCK_MISMATCH`(기록은 있는데 실제로 없음).

orchestrator_core 공개 API:
```python
OrchestratorCore(stations, params, robots, clock, next_goal_id=None)
on_order_board(run_mode, orders) # orders: list[OrderRecord(order_id, items: dict[str,int])]
on_dispatch_feedback(goal_id, robot_id, phase); on_dispatch_result(goal_id, success, robot_id, reason="")
on_fill_result(goal_id, success, placed: dict, missing: dict, reason="")
on_swap_result(goal_id, success, took_full, put_empty_done, reason="")
on_robot_removed(robot_id)
tick(active_robots) -> list ; order_of(robot) ; robot_view(robot)
```
명령: `DispatchCmd(goal_id, station_id, robot_id="")`, `CancelDispatchCmd(goal_id)`, `ReleaseCmd(robot)`, `RobotFaultCmd(robot, reason)`, `FillCmd(goal_id, robot, station_id, order_id, items)`, `SwapCmd(arm, goal_id, robot, station_id, order_id, take_full, put_empty)`, `ReportOrderCmd(order_id, robot, status, placed, missing, detail)`, CancelCmd, EventOut. 카운터 `OrchestratorCore.shipped`.

FMS(`FmsCore`, `fms_node`): 두 모듈을 묶어 명령을 서로에게 전달하고, 바깥 ROS 인터페이스를 맡는다. 중계 노드가 없으므로 합치기 전 판의 confirm_clear 콜백 그룹 문제는 사라진다.

## 6. 시나리오 형식 (`sim/scenarios/*.yaml`)
```yaml
seed: 7
speed_factor: 5.0
timeout_s: 900
stock: {arm1: {A: 6, B: 6}, arm2: {C: 6, D: 6}} # 실제 물건. 가짜 작업공간은 팔별로 나눠 둔다(실제 배치는 공유 영역 1곳, 관제 로직과는 무관)
inventory: {A: 6, B: 6, C: 6, D: 6} # 중앙의 재고 기록. 생략하면 stock 합계
orders:
  - {at_s: 0, id: O-001, items: {A: 1, C: 1}}
robots:
  pinky1: {start: P-6, speed_mps: 0.20, pose_noise_m: 0.03}
arm: {seconds_per_item: 4.0, swap_seconds: 8.0, fail_rate: 0.0}
dock: {duration_s: 3.0, error_cm: 1.0}
faults: [] # comm_loss, nav_abort, stuck 유지. 추가: arm_fail(대상 arm, count), swap_partial(count), operator_remove(로봇을 코스에서 치우고 confirm_clear 호출)
expect: {all_orders_finished: true, orders_shipped: 6, orders_partial: 0, orders_held: 0, fifo_violations: 0, dual_arm_overlaps: 0,
  zone_holder_max: 1, released_without_position_check: 0, unheld_zone_pose: 0, events_include: []}
```
`expect`의 숫자는 정확히 일치해야 통과한다. `orders_shipped_min`도 지원한다(고장 시나리오에서 "0건이어도 통과"가 되지 않도록 반드시 1 이상).

시나리오 6개: `basic_orders`(주문 6건, 고장 없음), `order_burst`(한꺼번에 8건, 진행 중 추가 주문, 밀린 주문이면 대기 구간을 거치지 않고 바로 작업공간으로 가는지 확인), `shared_lane_failure`(통신 두절+주행 실패), `stock_shortage`(재고 0인 품목과 기록만 있고 실물 없는 품목: PARTIAL과 HELD, OUT_OF_STOCK과 STOCK_MISMATCH 구분), `stuck_robot`(차선에서 멈춤: 뒤따르는 로봇이 막히고 경고가 나는지), `operator_recovery`(팔 고장 3회 → ERROR → 관리자가 치움 → 주문이 다시 처리되어 전부 끝남).

심판은 관제와 오케스트레이터가 내는 이벤트를 합쳐서 본다. 가짜 노드(FakePinky, FakeWorkcell, FakeArm3, FakeVision, FakeCentral)는 인터페이스가 같으므로 그대로 쓴다.

심판 추가 항목: `fifo_violations`(ORDER_BOUND 순서가 등록 순서와 다르거나 SHIPPED/PARTIAL 순서가 묶인 순서와 다르면 위반. ORDER_REQUEUED된 주문은 다음에 묶이는 주문이어야 하고, 되돌림 없이 두 번 묶이면 위반), `dual_arm_overlaps`(arm1과 arm2가 동시에 PLACING. 양쪽 팔이 필요한 주문인데 PLACING이 안 보이면 실패. 2026-11-04에 양팔 동시 계획으로 정했으므로 이 항목의 의미가 바뀐다: 동시 PLACING은 허용, 충돌만 위반. 가짜 workcell과 심판을 그에 맞춰 고치기 전까지 기대값은 참고용), `unexpected_errors`(시나리오가 예상하지 않은 ERROR 이벤트), `order_results`(주문별 결과), 시간 초과는 시나리오가 예상한 경우 외에는 실패.

## 7. 알려진 약점 (이번 작업에서 고치지 않고 문서에 남김)
- 단일 칸 구역 옆에서 3.5 cm 넘게 덜 가서 멈추면 뒤 구역이 해제되지 않아 조용히 멈출 수 있다. 가짜 로봇은 목표에 정확히 서므로 재현되지 않는다.
- `_send`에서 `server_is_ready()`가 false면 약 0.2초 만에 재시도를 소진해 ERROR가 되고, 재사용하려면 노드를 재시작해야 한다.
- `confirm_clear`에 없는 구역을 주면 KeyError로 노드가 죽을 수 있다.
- 교착 포기 대상 로봇의 위치 보고가 끊기면 해제에 필요한 보고가 모자라 `_finish_retreat`가 재확인을 반복한다. `comm_timeout_s`가 지나 `COMM_LOST`가 되면 그 로봇의 대기 요청이 모두 취소되어 원래 대기 고리가 끊어지고, 고리가 하나도 남지 않으면 `_check_deadlock`이 재확인 상태를 지운다. 다른 고리가 남아 있으면 재확인이 이어질 수 있고, 그동안 새 호출과 새 이동 단계도 나가지 않는다. 옛 입출고 코드에는 이 재확인을 끝내는 수정이 있지만 GG 코드에는 들어오지 않았다.
- 오케스트레이터가 멈추면 화면에 핑키 할당과 구역 상태, 관제 이벤트, 팔 상태, 상품 인식 결과가 오지 않고, `confirm_clear`와 STOP/PAUSE도 관제에 전달되지 않는다. 중앙이 직접 내는 이벤트는 계속 나온다. 중앙과 아래 계층의 직통을 없앤 대가다(4절). 비상 정지는 이 경로에 기대지 않도록 따로 정해야 한다(SR-36, 13절).
- `use_fake:=false`에서 `/central/orders` 발행자가 없으면 경고 없이 아무것도 움직이지 않는다.
- 이동/도킹 목표에 관제 쪽 시간 제한이 없어 결과가 안 오면 조용히 멈춘다(FSM 책임).
- 위치 잡음으로 조기 해제가 가능하다(필터 없음). 시험 무위반이 정지 상태 안전을 증명하지는 않는다.
- `robot_radius_m` 0.08은 트레이를 뺀 값이다. 트레이를 포함한 길이를 재서 칸 크기와 해제 여유를 다시 정해야 한다. 해제 여유는 칸 절반(0.165 m)보다 작아야 하므로, 지금 칸 크기로는 반경이 0.115 m 이상이면 관제가 시작을 거부한다.
- 오케스트레이터·관제 재시작 시 진행 중이던 주문과 할당을 기억하지 못한다.
- 4·5·5-1절은 분리 작업 후의 목표 상태다. 프로토타입 코드와 contract.yaml은 아직 관제 안에 주문 진행이 들어 있고 관제가 팔을 직접 부르는 이전 구조이며, 분리 후 시나리오 6개를 다시 통과시켜야 한다(11절).

## 8. 물리 위험 (실물 시험 전 미확인)
- 팔이 바구니+상품을 함께 들어야 한다. 그리퍼 20~45 mm, 물체 무게 약 150 g(02 문서에는 약 50 g), 실측 없음. 바구니 손잡이 형상과 총 무게 시험이 먼저다.
- 사용 도달 거리 약 20~25 cm라 팔 베이스를 도크에 바짝 붙여야 하고, 도크 정지 오차가 작아야 한다.
- 양팔 동시 궤적 계획(MoveIt 플래닝 그룹)과 `follow_joint_trajectory` 실행은 시험한 적이 없다. arm_control이 pymycobot으로 받은 궤적을 얼마나 따라가는지 실측이 없다.

## 9. 작업 규칙
- 실물 로봇 접속과 bringup 금지. git commit/push 금지. Confluence 수정 금지.
- ROS를 띄우는 모든 명령은 `ROS_DOMAIN_ID`를 아래 지정 값으로, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 격리한다(기본값 6은 로봇 네트워크).
- 코드 주석과 식별자는 영어, 기존 스타일 유지. README와 문서는 한국어.
- 프로토타입을 실행한 PC: Jazzy, colcon, pytest 7.4.4. ruff와 rmw_cyclonedds 없음. 실행과 CI 기준은 Fast DDS(`rmw_fastrtps_cpp`)다. 옛 결과는 WSL2+CycloneDDS였으므로 환경이 다르다고 밝힌다.
- 결과를 부풀리지 않는다. 실행 안 한 것은 안 했다고 쓴다.

## 10. HW와 실행 위치 (`09_시스템_아키텍처_v15.html` 기준)
| 장치 | 대수 | 구성 | 실행하는 소프트웨어 | 네트워크 |
|---|---|---|---|---|
| 관제 PC | 1대 이상 | 사양 미정 | 화면, central, orchestrator, fleet_manager + fleet_core | 유선 |
| 공유기 | 전용 1대 | 기종과 위치 미정 | 없음 | |
| Pinky Pro (pinky1~3) | 3대 | Raspberry Pi 4B(메모리 4 GB), 라이다 RPLidar C1, 구동 모터 Dynamixel 2개, 카메라 OV5647, 관성 센서 BNO055, 배터리 센서 ADC | pinky_bringup, pinky_fsm, pinky_dock, Nav2 + AMCL | Wi-Fi |
| JetCobot 작업공간 (arm1, arm2) | 2대 | MyCobot 280(6축 + 그리퍼), 카메라 640×480, 팔마다 팔 PC 1대(사양 미정) | 팔마다 arm_control, arm_vision. workcell은 두 팔 PC 중 1곳(13절) | 유선 |
| JetCobot 출고 구역 (arm3) | 1대 | 같은 구성 | arm_control(`swap_basket` 제공) | 유선 |

- 사양을 실물로 확인한 것은 핑키 1대(`pinky_06c5`)와 제트코봇 1대(`jetcobot_126b`)뿐이다. 나머지 4대는 같다고 가정했다.
- 전용 공유기 1대로 묶기로 했다(2026-11-04). 지금은 로봇마다 자기 핫스팟을 쓰고 있다.
- 실행 위치는 2026-11-04에 정했다: Nav2는 핑키 Pi, 상품 인식과 workcell은 팔 PC.

## 11. 코드의 현재 상태 (Drive `07_fleet_prototype/roscamp-repo-2`, 2026-11-04 확인)
**orchestrator라는 독립 구성 요소는 설계에만 있다. 그 기능(주문 묶기, 담기와 교환 호출, 바구니 상태, 최종 보고)은 코드에서 `FleetCore` 안에 들어 있다.** `pc/`에는 `central`(빈 패키지), `fleet_core`, `fleet_manager`만 있다. `fleet.py`와 `fleet_manager_node.py`에 orchestrator가 나오지 않고, `contract.yaml`에 `/fleet/dispatch`와 `/fleet/release`가 없다. `FleetCore._dispatch`와 `FleetManagerNode._dispatch`는 내부 함수이며 `/fleet/dispatch` 액션과 이름만 비슷하다.

| 항목 | 지금 코드 | 분리 후 (4·5·5-1절) |
|---|---|---|
| 주문 입력 | `fleet_manager_node`가 `/central/orders`를 구독해 `FleetCore.on_order_board`로 넘긴다 | orchestrator가 구독한다 |
| 핑키 호출 | `FleetCore._dispatch`가 묶이지 않은 주문 수만큼 `Dispatcher.pick`으로 고른다 | orchestrator가 호출 수를 정하고, fleet_core는 `/fleet/dispatch` goal마다 로봇을 고른다 |
| 주문 묶기 | `FleetCore._bind_next_order`(작업공간 도킹 결과에서 호출) | orchestrator가 dispatch 성공 결과를 받는 순간 |
| 담기, 교환 | `FleetCore`가 `FillCmd`, `SwapCmd`를 만들고 `fleet_manager_node`가 `/workcell/fill_basket`, `/arm3/swap_basket`을 직접 호출한다 | orchestrator가 호출하고 fleet은 팔을 모른다 |
| 팔 재시도, 시간 초과, 바구니 상태 | `FleetCore`(FILLING, SWAPPING, `_Mission.basket`) | orchestrator |
| 최종 보고 | `FleetCore`의 `ReportOrderCmd`를 `fleet_manager_node`가 `/central/report_order`로 보낸다 | orchestrator가 보낸다 |
| 고장 로봇의 주문 | `FleetCore.confirm_clear` 안에서 `ORDER_REQUEUED`를 낸다 | `confirm_clear` 응답이 accepted이거나 dispatch가 `ROBOT_REMOVED`로 실패하면 orchestrator가 되돌린다. 돌려줄 dispatch가 없는 경우(담기 중, 교환 중, 다음 명령 대기 중)도 여기에 들어간다(5-1절) |
| 중앙과 관제 사이 | `fleet_manager_node`가 `/fleet/assignments`, `/fleet/zones`, `/central/events`를 발행하고 `/fleet/confirm_clear`를 제공한다. 가짜 중앙은 `/fleet/*`를 구독하지 않고, 심판 노드 `scenario_monitor`가 이 토픽들을 구독하며 관리자 역할로 confirm_clear를 부른다 | 오케스트레이터를 거친다(4절). 관제의 이벤트 발행은 `/fleet/events`로 바뀌고, `scenario_monitor`는 `/orchestrator/confirm_clear`를 부른다 |
| 중앙과 팔 사이 | `contract.yaml`에 `/{arm}/state`, `/{arm}/vision/items`가 있다. 가짜 중앙은 둘 다 구독하지 않고, `scenario_monitor`가 심판으로 `/{arm}/state`를 구독한다 | 오케스트레이터가 `/orchestrator/{arm}/*`로 넘긴다. 이 변경으로 팔이 발행하는 토픽의 이름과 타입은 바뀌지 않는다 |
| 인터페이스 | `contract.yaml` 0.2-gg-draft. `/fleet/dispatch`, `/fleet/release`, `/fleet/events`, `/orchestrator/assignments`, `/orchestrator/zones`, `/orchestrator/{arm}/state`, `/orchestrator/{arm}/vision/items`, `/orchestrator/confirm_clear`, `/{arm}/follow_joint_trajectory`가 없다 | 추가한다 |
| 양팔 규칙 | `README.md`, `fake_workcell_node`, 심판 `dual_arm_overlaps`가 동시 PLACING을 위반으로 본다 | 동시 PLACING 허용, 충돌만 위반(6절) |
| 중앙, 핑키, 팔 | `pc/central`, `pinky/*`, `arm/*`는 빈 패키지이고 가짜 노드만 동작한다 | 각 담당이 구현한다 |
| GitHub | `addinedu-roscamp-16th/roscamp-repo-2`의 main에는 `README.md`와 `.gitignore`만 있다. 프로토타입은 푸시하지 않았다 | 첫 푸시는 04 문서 8절의 점검표를 마친 뒤 |

`08_무엇을_개발했나_설명`의 시험 527건과 시나리오 6개 결과는 이 이전 구조에서 얻은 값이다. 분리 후 구조의 결과로 인용하지 않는다.

분리 순서(제안):
1. `Dispatch.action`, `Release.srv`, `contract.yaml`을 담당자들과 합의한다.
2. `orchestrator_core` 테스트를 먼저 쓰고, `FleetCore`에서 주문 묶기, FILLING/SWAPPING, 팔 재시도, 바구니 상태, 보고를 옮긴다.
3. `fleet_manager`에 dispatch 서버와 release 서비스를 넣고, 팔 액션 클라이언트를 orchestrator 노드로 옮긴다.
4. 관제의 이벤트 발행을 `/fleet/events`로 바꾸고, orchestrator 노드에 관제·팔 상태와 confirm_clear 중계를 넣고, 응답이 accepted이면 `on_robot_removed`를 부른다. `scenario_monitor`가 부르는 confirm_clear를 `/orchestrator/confirm_clear`로 바꾼다. 실제 central은 아직 빈 패키지라 바꿀 코드가 없다.
5. 가짜 workcell과 심판을 양팔 동시 계획 기준으로 고친다.
6. 시나리오 6개와 기존 시험을 다시 통과시킨다.

## 12. Confluence SR과 다른 점 (819201 v2와 2026-11-04 대조)
기준 문서는 Confluence이고, 이 작업에서 Confluence는 고치지 않는다. 아래 항목은 어느 쪽으로 맞출지 정한 뒤 이 표에서 지운다.

| SR | SR 내용 | 이 문서와 코드 | 정할 것 |
|---|---|---|---|
| SR-22 (MUST) | 상품 놓인 곳과 바구니 위에는 각각 한 번에 한 팔만 들어간다 | 2절은 양팔 동시 궤적 계획을 쓰고 잠금 방식을 폐기했다(2026-11-04 결정). SR에는 반영되지 않았다 | SR을 고칠지, 한 팔씩 들어가는 방식으로 돌아갈지. 2026-11-05에 팀과 상의한다 |
| SR-13 (MUST) | 핑키 1대를 양보시켜 길 막힘을 자동으로 풀고, 안 되면 관리자에게 알린다 | 5절의 `DEADLOCK_RETREAT`는 아직 들어가지 않은 구역의 예약만 포기한다. 들어간 구역에서 물러나는 동작은 없다 | 이것으로 충족한다고 볼지, 양보 주행을 만들지. 구현 전이라 일단 둔다(2026-11-04) |
| SR-03 (MUST) | 주문 상태: 대기, 배정, 담는 중, 운반 중, 출고 완료, 보류, 실패 | 5-1절: PENDING, BOUND, FILLING, TO_OUTBOUND, SWAPPING, SHIPPED, PARTIAL, HELD. "실패"가 없고, SWAPPING과 PARTIAL이 SR에 없다 | 상태 대응표와 "실패"의 뜻 |
| SR-17 (MUST) | 핑키 상태: 대기, 이동, 도킹, 담기 대기, 출고 대기, 오류 + 원인 | `RobotState.STATE_*`: IDLE, MOVING, WAITING, DOCKING, DOCKED, ERROR. 담기 대기와 출고 대기가 구분되지 않고 오류 원인 필드가 없다 | 상수와 필드 |
| SR-36 (MUST) | 관리 프로그램과 별개의 비상 정지 | 이 문서와 `contract.yaml`에 경로가 없다. STOP은 화면, central, orchestrator를 거친다 | 경로와 듣는 쪽 |
| SR-27 (MUST) | 상품의 종류, 위치, 수량과 트레이·바구니 위치 인식 | `/{arm}/vision/items`는 arm1, arm2의 상품만 다룬다. 트레이·바구니 인식과 arm3의 인식 인터페이스가 없다 | 어디서 어떤 토픽으로 낼지 |
| SR-21 (MUST) | 핑키가 조금 어긋나게 서도 위치를 맞춰 담는다 | dispatch 결과에 도킹 오차가 있지만 `FillCmd`와 FillBasket goal에는 전달할 필드가 없다 | 오차를 넘길지, 카메라로 보정할지 |
| SR-31 (MUST) | 지도 위 핑키 위치 표시 | orchestrator를 거쳐 central로 올린다는 것만 정했고 토픽은 정하지 않았다 | 토픽 |
| SR-15 (SHOULD), UR-10 | 배터리가 낮으면 대기 구역으로 돌아가 충전 | `RobotState.battery` 필드만 있고 `fleet.py`에 판단이 없다 | 넣을지 |
| SR-09, SR-20, SR-23, SR-25, SR-28 (SHOULD) | 주문 취소, 사람 접근 대응, 작업 팔 거리 감시, 빈 바구니·출고 위치 관리, 담긴 개수 확인 | 이 문서에 전용 규칙과 인터페이스가 없다. `put_empty`와 `placed`는 있지만, 빈 바구니의 보관 수량 관리나 카메라로 하는 개수 확인은 아니다 | 어디까지 넣을지 |

## 13. 아직 정하지 않은 것
- **orchestrator 담당**: 핑키 관제 담당이 맡는 것으로 가정한다(2026-11-04, 팀 합의 전). `01_team_roles`의 6개 역할에는 아직 없다.
- dispatch의 이름, 필드, 취소 규칙: 신규 인터페이스라 담당자들과 합의해야 한다. `/fleet/events`와 `/orchestrator/*`의 이름도 함께 합의한다.
- STOP/PAUSE를 관제에 전달하는 방식: 별도 토픽 또는 dispatch 취소(4절). 어느 쪽이든 orchestrator가 보낸다. SR-34의 두 정지 방식은 PAUSE(하던 동작을 끝내고 멈춤)와 STOP(취소하고 바로 멈춤)에 대응한다.
- 화면에 표시할 핑키 위치(SR-31): orchestrator를 거쳐 central로 올린다. 토픽 이름과 필드는 미정이다.
- 비상 정지의 경로와 어느 노드가 듣는지(SR-36): 화면의 STOP과 달리 orchestrator에 기대지 않아야 한다.
- 화면에서 central로 가는 `confirm_clear`의 이름과 타입: 4절에 없다. 중앙 담당과 정한다.
- 치운 로봇을 다시 투입하는 절차: `fleet.py`에는 `OUT_OF_SERVICE`를 지우는 코드가 없다. 언제 어떻게 되돌릴지, 5-1절의 `basket=NONE`을 언제 EMPTY로 돌릴지 함께 정한다.
- arm3의 인식(SR-27): v15 HW 그림에는 arm3 카메라가 있지만 `arm_vision`과 `/{arm}/vision/items`는 arm1, arm2뿐이다.
- workcell이 arm1, arm2 중 어느 팔 PC에서 도는지: 팔 파트가 정한다.
- JetCobot의 Raspberry Pi 5를 어떻게 쓸지: 지금 구성은 팔을 팔 PC로 제어한다.
- 공유기 기종과 위치, 관제 PC와 팔 PC의 사양.
- 상품 영역의 정확한 위치, 출고 선반, 빈 바구니 공급 방법.
- 지도: `09_맵_아키텍처_결정_항목.txt`의 노드, 레인, 잠금 단위, 도크 접근, 트레이 포함 폭, 팔 위치.

## 14. 다른 문서와 다른 점 (이 문서들은 이번에 고치지 않았다)
| 문서 | 이 문서와 다른 내용 |
|---|---|
| `01_team_roles` | 관제가 주문을 묶고 담기와 교체 목표를 보낸다. orchestrator 담당이 없다. 작업공간 서버가 한 번에 1대만 넣는다. 팔 명령은 각 Pi에서 돈다고 적었다(10절은 팔 PC). |
| `02_프로젝트_설계_초안_GG` | orchestrator가 없고 관제가 `fill_basket`, `swap_basket`을 직접 호출한다. UR 2785281과 SR 62개를 기준으로 삼는다. |
| `03_fake_로봇_선개발_계획` | 관제가 workcell과 arm3를 직접 안다. 팔 2대는 한 번에 한 팔만 PLACING이다. |
| `04_GitHub_저장소_구성안` | `pc/`에 orchestrator 패키지가 없고, workcell 패키지 자리가 정해지지 않았다. |
| `08_무엇을_개발했나_설명` | 이전 구조의 코드와 시험 결과를 설명한다(11절). SR 62개 기준의 번호를 쓴다. |
| 저장소 `README`, `contract.yaml` | 11절의 이전 구조를 따른다. |
| `09_시스템_아키텍처_v15.html` | 이 문서와 같은 계층을 쓴다. 계층화 패턴으로 5개 계층을 나누어 그렸고, 화살표에는 전달 내용을 쉬운 말로 적었다. 토픽과 서비스 이름은 그림에서 빼고 이 문서 4절에 둔다. v14는 화살표에 글자가 없는 판이고, v13은 같은 구조를 계층 구분 없이 묶음 4개로 그린 판이며, v12는 UML 컴포넌트 다이어그램으로 그린 판이며, v11은 팔과 중앙의 직통이 남은 판이며, v10은 관제와 중앙의 직통도 남은 판이고, v9는 HW 그림이 이전 형식인 판이다. |
