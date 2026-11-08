# GG 공통 설계 (코드와 문서가 함께 따르는 기준, 초안)

기준은 Confluence에 있는 것이 최종입니다(2026-11-04 확정).
- 기준 UR: Confluence "주제1 : GG(Give Gift)" 페이지(425992, UR-01~14). 배터리·홈 복귀는 UR-10, 관리자 실시간 확인은 UR-11, 오류 알림은 UR-12, 충돌·간섭 방지는 UR-13, 이력 확인은 UR-14. 2785281(UR-01~15)은 더 이상 쓰지 않습니다.
- 기준 SR: Confluence "System Requirements" 페이지(819201, SR-01~38, UR 열 포함). UR 열의 "-"는 최종 UR에 대응 항목이 없다는 뜻입니다. "Copy of System Requirements"는 작업본이며 기준이 아닙니다.
- 11-06 결정(개선 지시서 `FMS_개선_지시서_2026-11-06.md`의 전제. 저장소 문서가 기준이고 Drive에는 아직 없다):
  1. 중앙이 레인 그래프 위에서 경로와 순서를 정한다. 자유 선 경로 + 연속 공간 예약(pinky_multi_sim 방식)은 쓰지 않는다. 전수 검사가 가능해야 하기 때문이다. 자유 선의 이점은 촘촘한 레인 그래프로 얻는다.
  2. 실행은 FollowPath(레인 폴리라인). 장애물 처리, 멈춤 감시, 대기-vs-우회, 끝 근처 재전송은 pinky_multi_sim의 규칙을 옮긴다. 라이다 합치기(obstacle_watch)는 옮기지 않는다. 감지는 로봇이 한다(14절).
  3. 충돌 회피는 시간 없는 구역 잠금이다. Open-RMF, 시간 궤적, MAPF, 우선순위 최적화는 안 한다.
  4. `orders`와 `traffic`의 분리를 지킨다. traffic은 주문을 모른다. 새 입력은 `commands.py`의 명령에 칸을 더한다.
  5. 11-05 밤 "정할 것 9가지"의 권장을 모두 채택한다: FollowPath, AMCL 1 cm(Nav2 담당), 물체는 멈추고 알리기 → 우회, FMS도 멈춤 감시, 같은 도메인 + namespace, 교착은 지금 방식, 레인 이탈은 멈추고 구역 유지, 관리자 재투입 명령, 잘못된 입력은 거부하고 계속 실행.
  6. 지도 요소는 방향별 레인 2줄의 넓은 길, 우회로가 있는 좁은 양방향 통로(+ 기존 교차)다. 레인 간격 ≥ 2r + 2e + 여유(e는 Gazebo 값 0.067 m 임시, 간격 0.3 m 이상). 긴 레인은 0.3~0.4 m 구간으로 쪼갠다.
  7. 우회 규칙은 결정적이다. 기다릴지 돌아갈지를 규칙 하나가 정한다.
  8. 검증: 특징 지도(구역 15개 안팎)에서 로봇 3대 전수 검사. 실제 지도는 2대 전수 + 3대 상한 탐색 + 시나리오.
- 사용자가 지시서를 바꾼 것: 이름에 `gg_` 접두사를 쓰지 않고(패키지는 0.5 이전 이름), 계약 버전은 `0.3-draft`이며, 핑키 계약은 제안이 아니라 관제가 요구하는 것으로 쓴다.
- 계층(2026-11-05, v17 FMS 통합 판, v18도 같다, 팀 합의 전): 사용자(화면) → 서버(central, FMS) → 로봇(핑키, 로봇팔). 위가 아래를 부르고 아래는 상태와 결과만 올린다. FMS는 프로세스 하나(노드 `fleet_manager`)이고, 그 안에 주문 진행을 맡는 `orders`와 교통 제어를 맡는 `traffic`이 파이썬 호출로 연결된다. 코드도 이 구조로 나눴다(11절). 이 문서와 Confluence가 다른 것으로 확인한 항목은 12절에 모았고, 어느 쪽으로 맞출지는 아직 정하지 않았다.
- 최종 갱신: 2026-11-06 밤(파트 8, 문서 동기화). 기준 트리는 `fms-work-order`의 `6ab04fe`이고, 개선 지시서 파트 0, 0.5, 1, 2, 4, 5, 5.5, 3, 7, 6이 모두 들어 있다. 앞서 지시서 0.5번에 따라 패키지·모듈 이름을 역할에 맞게 바꿨다(11절). 같은 날 앞서 코드 검토에서 찾은 결함 8건을 고치고 새 값과 이벤트를 반영했다(`docs/worklog_2026-11-06.md`). 2026-11-05: orchestrator와 fleet을 FMS 한 프로세스로 합치고, 둘 사이의 ROS 액션(`/fleet/dispatch`)과 서비스(`/fleet/release`), 중계 토픽(`/fleet/events`, `/orchestrator/*`)을 없앴다(4절). 지시서 "FMS 선행 작업"의 1~7번이 끝난 코드에 맞춰 전체를 고쳤다. 어긋났던 문장과 숫자의 근거는 `docs/worklog_2026-11-05.md`에 있다. 작업본은 저장소 `docs/GG_공통설계.md`이고 Drive 원본(2026-11-05 00:51 수정, 40 KB)은 고치지 않았다.

## 0. 용어와 컴포넌트

### 낱말

| 낱말 | 뜻 | 예 |
|---|---|---|
| 컴포넌트 | 설계에서 역할 하나로 부르는 덩어리. 프로세스 하나일 수도 있고 여럿일 수도 있다 | FMS, central, 핑키 |
| 노드 | ROS 2 그래프의 참가자. 토픽, 서비스, 액션은 노드가 주고받는다 | `fleet_manager`, `fake_pinky_node` |
| 프로세스 | 운영체제에서 실행 중인 프로그램 하나. 죽으면 그 안의 노드와 모듈이 함께 멈춘다 | `ros2 run fleet_manager fleet_manager_node` |
| 컴퓨터 | 프로세스가 도는 장치 | 관제 PC, 핑키의 Raspberry Pi 4B, 팔 PC |
| 모듈 | 프로세스 안의 파이썬 코드 묶음. 서로 파이썬 호출로 부르고 ROS를 쓰지 않는다 | `traffic`, `orders` |
| 플릿(fleet) | 핑키 3대 | `pinky1`~`pinky3` |
| 스테이션(station) | 작업공간과 출고. 도크와 대기 레인을 가진다 | `WORK`, `OUTBOUND` |
| 지점(waypoint) | 레인 그래프의 꼭짓점. ROS 노드와 헷갈리지 않게 "노드"라 부르지 않는다 | `W`, `P-4` |
| 레인(lane) | 지점에서 지점으로 가는 방향 있는 직선 | `Q`→`W` |
| 구역(zone) | 잠금 단위. 지점 하나와 그 지점으로 들어오는 레인들(3절). 코드와 심판이 쓰는 이름이다 | `W`, `AVENUE` |
| 도크(dock), 대기 지점(parking) | 스테이션에서 일하려고 서는 지점, 일이 없을 때 서는 지점 | `W`, `P-3`~`P-6` |

"칸", "셀", "그리드"는 옛 격자 지도의 낱말이라 지금 지도에는 쓰지 않는다.

FMS(fleet manager, RCS)는 핑키 플릿에 일을 나눠 주고(배분), 길을 정리하고(교통), 스테이션에 작업을 요청한다(핸드셰이크). 팔의 동작 자체는 스테이션이 한다. 프로세스 하나, 안에 orders와 traffic 두 모듈.

FMS는 컴포넌트 1개 = 프로세스 1개 = 노드 1개(`fleet_manager`, 실행 파일 `fleet_manager_node`)이고, 그 안에 모듈 2개(`orders`, `traffic`)와 둘을 잇는 `FleetManager`(`manager.py`)가 있다. v17·v18 그림은 FMS 안을 core, pinky, arm 세 모듈로 그렸다. core와 arm이 `orders`에, pinky가 `traffic`에 해당한다. 패키지 이름은 그대로이고 2026-11-06에 패키지 안의 모듈만 나눴다(11절).

### 컴포넌트

"시험"은 지금 그 자리에서 대신 도는 가짜 노드다. 표의 토픽과 서비스 중 `/fleet/robots`, `/fleet/heartbeat`는 팀 합의 전이다(4절).

| 컴포넌트 | 실체 | 어디서 | 갖는 상태 | 제공 | 호출, 구독 |
|---|---|---|---|---|---|
| 화면(고객, 관리자) | 없음. central과 HTTP로 잇는 것은 제안(v17) | 관제 PC(제안) | 없음 | 주문과 조작 입력, 상태 표시 | central |
| central | `pc/central`은 빈 패키지. 시험: `fake_central_node` | 관제 PC | 주문, 재고 기록, 이력. DB는 제안이고 central만 쓴다 | `/central/submit_order`, `/central/set_run_mode`, `/central/report_order` 서비스. `/central/orders`, `/central/events` 발행 | 계획: `/fleet/confirm_clear` 호출, `/fleet/robots`, `/fleet/assignments`, `/fleet/zones` 구독. `fake_central_node`는 이 중 아무것도 하지 않는다(11절) |
| FMS (`fleet_manager`) | `pc/fleet_manager`의 노드 1개와 `pc/fleet_core` | 관제 PC | 노드 자체가 가진 것은 진행 중인 목표 핸들, 결과를 기다리는 취소, 답을 받지 못한 보고, 액션 서버를 찾은 로봇 목록, 마지막으로 낸 `/fleet/zones`와 `/fleet/assignments`, 시작 침묵 기한이다. 나머지는 아래 두 모듈. 프로세스가 죽으면 전부 사라진다(7절) | `/fleet/robots`, `/fleet/assignments`, `/fleet/zones`, `/fleet/heartbeat`, `/central/events` 발행. `/fleet/confirm_clear`, `/fleet/reinstate`(제안) 서비스 | `/{pinky}/move_to`, `/{pinky}/dock`, `/workcell/fill_basket`, `/arm3/swap_basket` 액션. `/central/report_order` 서비스. `/central/orders`, `/{pinky}/state` 구독 |
| `orders` | 모듈 `fleet_core/orders/orchestrator.py`(`OrderOrchestrator`) | FMS 안 | 대기 주문, 로봇별 묶인 주문과 담은 것·결품, `basket[robot]`, 팔 작업과 재시도 횟수, `ARM_NO_RESULT`를 낸 뒤 늦은 결과를 기다리는 팔 작업 | `on_order_board`, `on_fill_result`, `on_swap_result`, `on_dispatch_feedback`, `on_dispatch_result`, `on_robot_removed` | 명령을 내놓는다: `DispatchCmd`, `CancelDispatchCmd`, `ReleaseCmd`, `RobotFaultCmd`, `FillCmd`, `SwapCmd`, `ReportOrderCmd` |
| `traffic` | 모듈 `fleet_core/traffic/`: `controller.py`(`TrafficController`), `mission.py`(로봇별 기록), `recovery.py`(통신 두절, 복구, 교착 후퇴, confirm_clear), `blocking.py`(막힘), `detour.py`(우회), `isolation.py`(자동 격리), `off_lane.py`(레인 이탈), `lane_path.py`(이동 목표 묶음과 레인 폴리라인), `release_floor.py`(해제 보강), `zones.py`(`ZoneLocks`), `selector.py`(`RobotSelector`), `deadlock.py`, `lane_map.py`(`LaneMap`) | FMS 안 | 로봇별 임무 단계, 경로, 진행 중 목표, 고장. 구역 잠금과 대기열. 열린 호출. 주문과 바구니는 모른다 | `on_robot_state`, `on_move_result`, `on_goal_rejected`, `on_dock_result`, `on_dispatch`, `on_dispatch_cancel`, `on_release`, `on_run_mode`, `on_robot_fault`, `confirm_clear` | 명령을 내놓는다: `MoveCmd`, `DockCmd`, `CancelCmd`, `DispatchFeedback`, `DispatchResult` |
| `FleetManager` | 클래스 `fleet_core/manager.py` | FMS 안 | 목표 번호, 호출 하나 동안 고정된 시각, 기록 대상 | 노드가 부르는 입구. `tick()`, 뷰, `record_to` | 두 모듈 |
| 핑키 FSM | `pinky/pinky_fsm`은 빈 패키지. 시험: `fake_pinky_node` | 핑키 Pi 4B | 진행 중인 목표, FSM 상태, 위치 | `/{pinky}/move_to`, `/{pinky}/dock` 액션. `/{pinky}/state` 발행 | `/{pinky}/navigate_to_pose`(Nav2), pinky_dock. `/fleet/heartbeat` 구독 |
| 핑키 내부 | `pinky_dock`은 빈 패키지, `pinky_nav_config`는 자리표시 launch뿐이다. Nav2 + AMCL, `pinky_bringup`은 제조사 패키지 | 핑키 Pi 4B | 지도 위 위치 | `/{pinky}/navigate_to_pose`, `/odom`, `/scan` | `/cmd_vel` |
| workcell | 없음. 시험: `fake_workcell_node`. 구조는 팔 담당이 정한다. v17은 FMS의 arm 모듈과 `arm_planner`(임시 이름, 제안)로 그렸다 | 팔 PC 중 한 곳 | 두 팔의 작업과 재고 영역 | `/workcell/fill_basket` 액션 | arm_control |
| arm_control | `arm/arm_control`은 빈 패키지. 시험: `fake_arm_node`(arm3), arm1과 arm2는 `fake_workcell_node`가 함께 맡는다 | 팔 PC | 팔 상태 | `/arm3/swap_basket` 액션. `/{arm}/state` 발행 | 팔 하드웨어 |
| arm_vision | `arm/arm_vision`은 빈 패키지. 시험: `fake_vision_node`. Vision 서버로 옮기는 것은 제안(v17) | 팔 PC | 인식 결과 | `/{arm}/vision/items`(arm1, arm2) 발행 | 카메라 |
| 시험 전용 | `sim_clock_node`, `scenario_monitor_node`(심판) | 관제 PC, 시험 때만 | 시뮬 시각, 판정 기록 | `/clock`, `summary.json`, `safety.json` | 관리자 역할로 `/fleet/confirm_clear`, `/fleet/reinstate`. 첫 치우기는 코스 밖 확인 뒤 `settle_s` 0.5초가 지나고 보유가 `quiet_s` 0.3초 그대로일 때, 늦어도 `act_by_s` 1.0초에 보낸다(시뮬레이션 초). `ZONES_STILL_HELD`면 남은 구역을 다시 치우고 심판이 아는 남은 구역(보유 스냅샷과 `ZONES_STILL_HELD`)이 없고 응답 뒤 2초가 지났을 때만 재투입한다(`operator_clear.py`). 알림이 그보다 늦으면 재투입이 거부될 수 있고 다시 부르지 않는다 |

### 상태 값

| 값 | 어디 | 뜻 |
|---|---|---|
| `IDLE` | 임무 단계 | 지도에 올랐고 갈 곳이 없다. 호출이 있으면 가고 없으면 대기 지점으로 향한다 |
| `PARKED` | 임무 단계 | 대기 지점에 있다. 더 안쪽 지점이 비면 전진한다 |
| `TO_PARK`, `TO_WORK`, `TO_OUTBOUND` | 임무 단계 | 대기 지점, 작업공간, 출고 구역으로 이동 중 |
| `DOCKING` | 임무 단계 | dock 목표 실행 중 |
| `AT_STATION` | 임무 단계 | 도크에 서서 `on_dispatch`(robot_id 지정)나 `on_release`를 기다린다 |
| `NONE` | `fault` | 문제 없음 |
| `ERROR` | `fault` | 지도에 오른 로봇이면 격리됐다(5절 자동 격리). 구역은 쥐고 주문은 대개 이미 되돌렸다. 예외: 시작 때 `PLACEMENT_CONFLICT`는 지도에 없어 격리하지 않는다. `ARM_NO_RESULT`와 격리 때 교환 중이던 로봇은 결과나 `confirm_clear`까지 주문을 쥔다(5-1절). 관리자는 치우고 `confirm_clear`만 한다. 이유는 `fault_reason` |
| `COMM_LOST` | `fault` | 보고가 `comm_timeout_s` 동안 없다. 판단은 멈추지 않고 그 로봇의 요청만 취소한다 |
| `RECOVERING` | `fault` | 보고가 돌아왔고 `recover_check_s` 뒤에 위치를 확인한다(`fault_reason` CHECKING_POSE). 또는 레인을 벗어나 세웠고 돌아오기를 기다린다(`fault_reason` OFF_LANE, 5절). 둘이 겹치면 통신 쪽을 보인다 |
| `OUT_OF_SERVICE` | `fault` | 관리자가 치웠다(`confirm_clear`). `/fleet/reinstate`로 되돌린다(4절, 5절) |
| `IDLE`, `MOVING`, `WAITING`, `DOCKING`, `DOCKED`, `ERROR`, `BLOCKED`, `LOCALIZING` | `fsm_state` | 핑키가 보고한 값 그대로. 임무 단계와 이름이 같아도 다른 값이다. `BLOCKED`는 경로 앞 물체로 섰다, `LOCALIZING`은 위치를 아직 못 잡았다는 뜻이다(핑키 계약 3·4절) |
| `TO_WORK`, `FILL`, `TO_OUTBOUND`, `SWAP`, `WAIT` | `task_type` | `/fleet/assignments`와 `/fleet/robots`의 일 종류 |
| `PENDING`, `BOUND`, `FILLING`, `TO_OUTBOUND`, `SWAPPING`, `SHIPPED`, `PARTIAL`, `HELD` | 주문 상태 | 5-1절 |
| `EMPTY`, `FULL`, `NONE` | `basket[robot]` | 빈 바구니, 담긴 바구니, 바구니 없음 |
| `RUN`, `PAUSE`, `STOP` | run_mode | 5절, 5-1절 |

`/fleet/robots`의 `fault_reason` 값(`controller.py`의 `status_view`). 보이는 순서는 고장(`ERROR`, `OUT_OF_SERVICE`), 통신(`COMM_LOST`, `RECOVERING`), 레인 이탈(`RECOVERING`), 막힘이다(`off_lane.py`의 `_shown_fault`). `fault`가 NONE이면 막힘 동안만 `BLOCKED`이고 그 밖에는 빈 값이다.

| `fault` | `fault_reason` | 언제 |
|---|---|---|
| `ERROR` | `STALLED` | 멈춤 감시로 재시도를 다 썼다(`GOAL_FAILED`) |
| `ERROR` | `NOT_AT_TARGET` | 성공 결과 뒤 위치가 목표에서 멀어 재시도를 다 썼다 |
| `ERROR` | `REJECTED`, `DOCK_FAILED` | 목표 거절, 도킹 실패로 재시도를 다 썼다 |
| `ERROR` | 이동 결과의 reason(`NAV_ABORTED`, `TIMEOUT`, `CANCELED` 등) | 이동 실패로 재시도를 다 썼다. 핑키가 보낸 값 그대로 |
| `ERROR` | `BLOCKED` | 막힘이 `blocked_timeout_s`에 닿았다(`BLOCKED_TOO_LONG`) |
| `ERROR` | `RECOVERY_CONFLICT` | 복구한 로봇이 남의 구역 안에 있다 |
| `ERROR` | `PLACEMENT_CONFLICT` | 시작 때 자리 충돌. 놓이면 지운다 |
| `ERROR` | `OFF_LANE` | 레인을 벗어나 세운 뒤 `off_lane_timeout_s` 안에 돌아오지 않았다(파트 3) |
| `ERROR` | 주문 쪽 이유(`<서버>: <reason>`, `<서버> goal <n> never reported`, 일부 담김 금지 detail) | `RobotFaultCmd`(팔 재시도 초과, `ARM_NO_RESULT`, `ship_partial` false) |
| `OUT_OF_SERVICE` | `confirm_clear`의 note | 관리자가 치웠다 |
| `COMM_LOST` | `NO_STATE` | 보고가 `comm_timeout_s` 동안 없다 |
| `RECOVERING` | `CHECKING_POSE` | 복구 확인 중 |
| `RECOVERING` | `OFF_LANE` | 레인을 벗어나 세웠고 돌아오기를 기다린다(파트 3). 통신이 정상일 때만 이렇게 보인다 |
| `NONE` | `BLOCKED` | 막힘을 연 동안(5절 레인 막힘) |
| `NONE` | 빈 값 | 문제 없음 |

### 이벤트 코드 (`/central/events`)

수준은 INFO, WARN, ERROR이다. 심판은 시나리오가 예상하지 않은 ERROR를 실패로 센다.

| 코드 | 수준 | 내는 곳 | 뜻 |
|---|---|---|---|
| `DOCKED` | INFO | traffic | 도킹 끝. detail에 오차 |
| `GOAL_RETRY` | WARN | traffic | 이동이나 도킹 목표가 실패해 다시 보낸다 |
| `GOAL_FAILED` | ERROR | traffic | 재시도를 다 썼다 |
| `GOAL_STALLED` | WARN | traffic | 목표가 열린 채 `stall_s`(도킹은 `dock_stall_s`) 동안 제자리라 취소하고 같은 목표를 다시 보낸다. 시도 한 번으로 센다(11-06, 2번) |
| `GOAL_NOT_SENT` | WARN | traffic | 액션 서버가 없어 이동이나 도킹 목표를 보내지 못했다. 시도 횟수에 넣지 않고 다시 보낸다. 목표가 서버에 닿을 때까지 한 번만 낸다 |
| `HEARTBEAT_LOST` | WARN | traffic | 로봇이 heartbeat를 못 받아 스스로 이동을 끝냈다(결과 reason). 시도로 세지 않고 `move_retry_wait_s` 뒤 같은 목표를 다시 보낸다(11-06, 4번) |
| `LANE_BLOCKED` | WARN | traffic | `BLOCKED` 보고가 `blocked_mark_s` 동안 이어져 달리던 레인을 막힘으로 표시하고 목표를 취소했다. detail에 레인(11-06, 4번) |
| `LANE_CLEARED` | INFO | traffic | 표시한 레인이 풀렸다(`BLOCKED`가 아닌 보고가 이어짐, 또는 `confirm_clear`). 같은 목표를 다시 보낸다 |
| `DETOUR` | INFO | traffic | 다음 구역을 청할 때 더 싼 길로 바꿨다(11-06, 5번). detail에 피한 것(구역과 보유자와 서 있는 시간, 또는 막힌 레인)과 새 경로 |
| `BLOCKED_TOO_LONG` | ERROR | traffic | `BLOCKED`로 보고된 시간이 `blocked_timeout_s`에 닿았다. `fault_reason=BLOCKED`, 구역은 쥔다 |
| `COMM_LOST` | WARN | traffic | 보고가 `comm_timeout_s` 동안 없다. 지도에 오른 로봇이 `LOCALIZING`만 보내는 것도 같다(detail에 적는다) |
| `COMM_RESTORED` | INFO | traffic | 보고가 돌아왔다. 위치를 확인하고 이어 간다 |
| `ZONE_WAIT_LONG` | WARN | traffic | 구역 하나를 `zone_wait_warn_s` 넘게 기다린다 |
| `ZONES_CLEARED` | INFO | traffic | `confirm_clear`로 잠금을 풀었다 |
| `CLEAR_REFUSED` | WARN | traffic | 이름은 모두 맞는데 `confirm_clear`를 거부했고 아무것도 바꾸지 않았다(구역을 쥔 로봇인데 목록이 빔). 이유는 detail과 응답의 `reason`(관제 요구, 팀 합의 전, 4절)에 적는다 |
| `INPUT_REJECTED` | WARN | FMS(`FleetManager`, confirm_clear는 traffic) | 바깥 입력을 거부하거나 버렸다. detail에 무엇을 왜. 대상: 모르는 로봇이나 구역의 `confirm_clear`(응답 `accepted=false`, `reason`), 모르는 로봇·`OUT_OF_SERVICE`가 아닌 로봇·쥔 구역이 남은 로봇·틀린 basket의 `reinstate`(같은 응답), 모르는 로봇의 goal 결과, `/{pinky}/state`의 robot_id 불일치·유한하지 않은 pose·지도 밖(`input_pose_margin_m`) 보고, `/central/orders`의 틀린 run_mode(메시지를 버림)나 틀린 주문(run_mode는 적용, 주문은 이전 것 유지). 서비스는 호출마다 한 번, 토픽은 출처마다 한 번 내고 정상 메시지가 오면 다시 낸다. 늦게 온 결과의 모르는 goal ID는 정상이라 내지 않는다 |
| `ROBOT_REINSTATED` | INFO | FMS(`FleetManager`) | `/fleet/reinstate`를 받아들였다. detail에 바구니와 note. 로봇은 다음 위치 보고부터 시작과 같은 절차로 맵에 오른다(이동·도킹 중이면 기다리고, 배치 충돌이면 그 구역 주인만 세운다, 5절) |
| `ZONES_STILL_HELD` | WARN | traffic | `confirm_clear` 뒤에도 그 로봇이 쥐고 있는 구역이 남았다. 다시 호출해야 풀린다 |
| `LANE_ERROR` | WARN | traffic | 이동 목표 중에 지금 달리는 레인에서 `lane_error_warn_m` 넘게 벗어난 보고가 `release_confirm_samples`번 이어졌다. 목표를 취소해 세우고 구역은 쥔다. 세울 때마다 한 번 낸다(11-06, 3번) |
| `BACK_ON_LANE` | INFO | traffic | 레인을 벗어나 세운 로봇의 보고가 그 레인 `half_width_m` 안으로 `release_confirm_samples`번 이어졌다. 같은 목표를 다시 보낸다 |
| `OFF_LANE` | ERROR | traffic | 레인을 벗어나 세운 로봇이 `off_lane_timeout_s` 안에 돌아오지 않았다. `fault=ERROR`, `fault_reason=OFF_LANE`, 구역은 쥔다. 자동 격리(`ROBOT_ISOLATED`)로 이어진다. 관리자가 본다 |
| `DEADLOCK_DETECTED` | ERROR | traffic | 대기 고리를 찾았다 |
| `DEADLOCK_RETREAT` | ERROR | traffic | 로봇 1대가 아직 들어가지 않은 구역을 포기한다 |
| `DEADLOCK_UNRESOLVED` | ERROR | traffic | 풀지 못했다. 관리자가 봐야 한다 |
| `PLACEMENT_CONFLICT` | ERROR | traffic | 시작 때 다른 로봇이 쥔 구역이나 그 충돌쌍에 서 있다. 해결될 때까지 전원이 기다린다. 지도에 없으므로 격리하지 않는다. 재투입한 로봇이면 그 구역들의 주인만 세운다(5절, detail에 이름) |
| `RECOVERY_CONFLICT` | ERROR | traffic | 복구한 로봇이 다른 로봇이 쥔 구역 안에 있다. 그 로봇을 격리한다 |
| `ROBOT_ISOLATED` | ERROR | traffic | 지도에 오른 로봇이 ERROR가 되어 자동 격리했다(11-06, 5.5번). 구역은 쥐고 열린 요청과 목표는 취소했다. detail은 이유로 시작하고 쥔 구역을 적는다. ERROR가 될 때 한 번 낸다 |
| `ROBOT_NOT_REPORTING` | ERROR | traffic | `startup_wait_s`가 지나도 위치를 보고하고 멈춰 서지 않은 로봇이 있다(`PLACEMENT_CONFLICT`로 거부된 로봇은 빼고 센다). 전원이 기다린다 |
| `ORDER_BOUND` | INFO | orchestrator | 주문을 로봇에 묶었다. detail은 주문 ID |
| `ORDER_REQUEUED` | INFO | orchestrator | 격리된 로봇(사람 없이 바로), 또는 격리 없이 치운 로봇의 주문을 대기 맨 앞으로 되돌렸다. detail은 주문 ID로 시작하고 바구니에 담겨 있던 품목을 적는다. 격리 때는 `robot isolated: <이유>`가 붙는다 |
| `FILL_RETRY`, `SWAP_RETRY` | WARN | orchestrator | 팔 작업이 실패해 다시 보낸다 |
| `FILL_FAILED`, `SWAP_FAILED` | ERROR | orchestrator | 재시도를 다 썼거나 담은 일부를 내보낼 수 없다 |
| `ARM_NO_RESULT` | ERROR | orchestrator | 취소한 팔 작업의 결과가 끝내 안 왔다. 바구니 내용을 알 수 없다 |
| `ARM_LATE_RESULT` | WARN | orchestrator | `ARM_NO_RESULT`를 낸 작업, 또는 로봇을 치울 때 취소한 작업(무엇을 했다고 할 때만)의 결과가 늦게 왔다. detail에 보고 내용을 적는다. 로봇을 치운 뒤라면 주문이 두 번 담기거나 출고될 수 있다(5-1절) |
| `CANCEL_UNCONFIRMED` | WARN | `fleet_manager` 노드 | 취소를 요청하고 `cancel_warn_after` × `cancel_retry_s`가 지나도 목표의 결과가 없다. 서버가 목표를 아직 수락하지 않아 취소를 보내지 못한 경우도 포함한다. source는 로봇이나 서버, detail에 목표 ID. 목표당 한 번 낸다. 취소는 결과가 올 때까지 계속 보내되, 결과를 잃은 이동·도킹 취소(결과 future 오류)는 이 경고 다음 재시도에서 그만둔다 |
| `ORDER_REPORT_REFUSED` | ERROR | `fleet_manager` 노드 | central이 주문 보고에 `accepted=false`로 답했다. detail은 주문 ID로 시작한다. 다시 보내지 않는다 |
| `NO_ROBOT_AVAILABLE` | WARN | orchestrator | 주문은 있는데 지도에 올라 고장이 아닌 로봇이 없다. 전원이 통신 두절일 때는 돌아올 수 있으므로 내지 않는다 |
| `ORDER_ACCEPTED`, `ORDER_SHIPPED` | INFO | central | 주문을 받았다, 출고했다 |
| `ORDER_REJECTED`, `ORDER_PARTIAL`, `ORDER_HELD` | WARN | central | 거절, 일부 출고, 보류 |
| `OUT_OF_STOCK`, `STOCK_MISMATCH` | WARN | central | 재고 기록도 0, 기록은 있는데 실제로 없음 |

### 파라미터 (`pc/fleet_core/fleet_core/data/fleet_params.yaml`)

전부 실물로 재기 전의 값이다. 표의 "추정"은 파일에도 그렇게 적혀 있다는 뜻이다.

| 이름 | 값 | 뜻 |
|---|---|---|
| `basket_capacity` | 4 | 바구니 하나에 담는 최대 개수. 추정 |
| `ship_partial` | true | 일부만 담겨도 출고한다 |
| `robot_radius_m` | 0.08 | 몸체 반경 r. 추정, 트레이를 뺀 값 |
| `release_margin_extra_m` | 0.05 | 심판 전용. r + 이 값이면 코스 밖이다 |
| `release_distance_m` | 0.2 | 해제 거리 d(5절). 근거식 d ≥ r + ε + v·(τ + tick), d ≥ 2r + ε. 실측 전 값이고 압박 시험 표는 작업 기록 7번에 있다 |
| `release_confirm_samples` | 3 | 해제 전에 연속으로 확인하는 보고 수 |
| `lane_error_warn_m` | 0.15 | `LANE_ERROR`를 내고 로봇을 세우는 이탈 e |
| `move_retry_limit` | 2 | 첫 실패 뒤 재시도 수. 이동, 도킹, 팔 작업이 같이 쓴다 |
| `move_retry_wait_s` | 2 | 이동이나 도킹 실패 뒤 다시 보내기까지. 추정. 세 번 시도하는 시간이 `comm_timeout_s`보다 길어 통신 두절이면 `COMM_LOST`가 먼저 난다 |
| `stall_s` | 6 | 이동 목표가 열린 채 이만큼 제자리면 멈춤. pinky_multi_sim(Gazebo 정리 4.3)과 같은 값. `comm_timeout_s`보다 길어 말 없는 로봇은 `COMM_LOST`가 먼저 난다 |
| `stall_move_m` | 0.05 | 매끄럽게 한 위치가 이보다 덜 움직이면 제자리. 지시서와 pinky_multi_sim은 0.02였지만, 위치 잡음 σ 0.01 m부터 감시가 걸리지 않아 올렸다(작업 기록 11-06 파트 2). 0.01 m/s보다 느린 로봇은 멈춤으로 본다 |
| `stall_smooth_s` | 1 | 멈춤 감시가 보는 위치의 지수 평균 시간 상수. 잡음 하나로 시계가 다시 시작되지 않게 한다 |
| `dock_stall_s` | 15 | 도킹 목표의 멈춤 시간. 도킹은 천천히 맞추므로 더 길다. 추정 |
| `blocked_mark_s` | 5 | `BLOCKED`가 이만큼 이어지면 레인을 막힘으로 표시하고 목표를 취소한다. 추정(핑키 계약 4절) |
| `blocked_timeout_s` | 60 | `BLOCKED`로 보고된 시간이 이만큼이면 ERROR(`BLOCKED`). 추정 |
| `blocked_rejoin_s` | 10 | 풀린 뒤 이 안에 같은 레인에서 다시 막히면 막힌 시간을 이어 센다. 5초 표시와 60초에 같은 시계를 쓴다 |
| `queue_gap_m` | 0.6 | 경로상 같은 레인이나 다음 레인 앞 이 거리 안에 정상 로봇이 있으면 `BLOCKED`를 줄 서기로 본다. 추정 |
| `detour_enabled` | true | 우회 규칙(5절)을 쓴다. false면 지금처럼 FIFO로 기다린다(비교용). 이 값이 없는 옛 기록은 false로 재생한다(데이터클래스 기본값) |
| `station_wait_estimate_s` | 30 | 스테이션, 대기 칸, 할 일 대기로 선 로봇이 비키기까지의 예상 시간. 추정 |
| `detour_speed_mps` | 0.2 | 예상 대기 시간을 거리 벌점으로 바꾸는 속도. 추정, FakePinky와 시나리오의 기본 속도 |
| `off_lane_timeout_s` | 20 | 레인을 벗어나 세운 로봇이 이 안에 돌아오지 않으면 `OFF_LANE`(ERROR). 추정 |
| `arrival_tolerance_m` | 0.15 | 이동의 성공 결과를 도착으로 치는 목표 지점까지의 거리. 추정. 가장 짧은 레인에서 `release_distance_m`을 뺀 값(기본 지도 0.2 m)보다 작아야 한다(7절) |
| `arrival_settle_s` | 1 | 성공 결과가 위치 보고보다 먼저 왔을 때 보고를 기다리는 시간. 추정. 보고 지연 τ보다 길어야 한다 |
| `startup_wait_s` | 10 | `ROBOT_NOT_REPORTING`을 내기까지 |
| `comm_timeout_s` | 3 | `COMM_LOST`로 보기까지. 추정 |
| `heartbeat_period_s` | 0.5 | `/fleet/heartbeat` 주기. 관제 요구(팀 합의 전), 벽시계 |
| `heartbeat_timeout_s` | 1.5 | 로봇이 멈추기까지. 관제 요구(팀 합의 전), 벽시계 |
| `recover_check_s` | 1 | 통신 복구나 교착 포기 뒤 위치를 확인하기까지. 추정 |
| `arm_cancel_wait_s` | 5 | 취소한 팔 작업의 결과를 기다리는 시간. 추정 |
| `arm_retry_wait_s` | 2 | 팔 실패 뒤 다시 보내기까지. 추정 |
| `fill_timeout_s` | 90 | 담기 시간 제한. 추정 |
| `swap_timeout_s` | 60 | 교환 시간 제한. 추정 |
| `cancel_retry_s` | 1 | 취소한 목표의 결과가 없을 때 취소를 다시 보내는 간격. 추정 |
| `cancel_warn_after` | 5 | `CANCEL_UNCONFIRMED`를 내기까지 결과 없이 지나간 재시도 주기 수. `cancel_retry_s`를 곱한 값이 `arm_cancel_wait_s` 이상이어야 코어가 아직 기다리는 팔을 늦었다고 하지 않는다(시험으로 고정) |
| `report_retry_s` | 2 | 답이 없는 주문 보고를 다시 보내기까지. 추정 |
| `zone_wait_warn_s` | 60 | `ZONE_WAIT_LONG`을 내기까지 |
| `deadlock_grace_s` | 5 | 대기 고리를 찾고 포기 대상을 정하기까지 |
| `score_same_call` | -4 | 이미 호출된 로봇의 점수 보정. 다른 로봇이 뚜렷이 가깝지 않으면 호출을 유지한다 |
| `score_distance_weight` | 0.91 | 레인을 따른 거리 1 m당 점수 |
| `priority_at_station`, `priority_has_cargo` | 200, 100 | 교착 때 포기 대상을 고르는 우선순위. 스테이션에 서 있는지, 이름 지정 요청을 받았는지 |
| `input_pose_margin_m` | 1.0 | 보고 위치가 웨이포인트들의 범위(최소·최대 x, y)에서 이보다 멀면 그 보고를 버리고 `INPUT_REJECTED`. 지시서의 "음수 좌표"를 지도 밖으로 넓혔다. 실제 지도 좌표는 음수일 수 있다 |
| `lane_goal_mode` | path | 이동 목표 모드(5절). `path`는 레인 폴리라인을 `MoveTo.path`로 보낸다(FollowPath). `point`는 같은 방향 레인 끝 지점 하나만 보낸다(호환·비교용). 이 값이 없는 옛 기록은 `point`로 재생한다(데이터클래스 기본값) |
| `release_clear_of_floor` | true | 해제 보강(5절, 파트 7). 경로 위 해제에 "구역의 다른 바닥 선에서 중심선 거리 d 이상"을 더하고, 시작 때 구역을 떠나는 레인의 끝이 그 기준을 넘는지 본다(어기면 시작 거부). 이 값이 없는 옛 기록은 예전 규칙으로 재생한다 |

지도 파일의 값: `half_width_m`(0.05, 레인에서 이만큼까지 레인 위로 친다. 추정), `lane_entry_m`(0.25), `bundle_two_way`(true, 양방향 구간을 잠금 하나로 묶는다).

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

## 3. 지도 (`pc/fleet_core/fleet_core/data/lane_map.yaml`, 임시)
이 배치는 임시다(2절). 실제 지도가 나오면 이 파일만 바꾼다. 2026-11-05 맵 아키텍처 논의에서 정할 항목은 `09_맵_아키텍처_결정_항목.txt`에 있다.

**로봇은 지점 사이를 어느 방향으로든 직선으로 간다. 대각선을 못 가는 점유 격자가 아니다.** 일방통행 한 바퀴였던 이전 지도에는 교착이 생길 곳이 없어, 양방향 구간과 교차가 있는 지도로 바꿨다(사용자 결정, 2026-11-05). 11-05 PDF는 지도 회의에 요소 하나만 제안하자고 했고 이 지도에는 둘이 있다.

지도 파일: 지점(`waypoints:`, id, x, y), 방향 있는 레인(from, to, 길이를 안 적으면 좌표로 계산), `two_way`(양방향 구간), 스테이션(dock, queue, commit, server, arms, action), parking(대기 지점, 운전 순서대로).

기본 지도: 지점 14개, 방향 있는 레인 17개, 구역 13개. 스테이션 `WORK`(도크 W, 대기 레인 L→Q, Q→W, 서버 `workcell`, 팔 arm1·arm2, 동작 `fill`), `OUTBOUND`(도크 O, 대기 레인 R→QO, QO→O, 서버 `arm3`, 동작 `swap`). 대기 지점 P-3~P-6. 양방향 구간 `AVENUE`(A↔B, 1.1 m). N→S가 대로를 지점 없이 가로지른다.

- **구역(잠금 단위):** 지점마다 구역 하나다. 레인은 끝 지점의 구역에 속한다. 그래서 한 지점으로 들어오는 레인들은 잠금 하나와 FIFO 하나를 함께 쓴다. 들어오는 레인이 하나면 레인 하나가 잠금 하나다.
- **양방향 구간:** `two_way`로 적은 한 쌍(a→b, b→a)은 두 끝 지점의 구역을 하나로 합친다(`bundle_two_way`, 전수 검사에서 끌 수 있다). 같은 쌍을 레인 두 줄로 적으면 지도를 거부한다.
- **충돌쌍:** 두 구역의 레인이 2r보다 가까이 지나면 동시 점유 금지 쌍으로 자동 계산한다. 지점을 공유하는 레인은 그 지점을 떠나는 쪽의 처음 d를 빼고 잰다. 앞 로봇은 d를 지나야 뒤 구역을 놓으므로 그 구간은 겹칠 수 없기 때문이다. 양방향 쌍 자신은 뺀다. 기본 지도의 충돌쌍은 `{AVENUE, S}` 하나다.
- 비교용 지도는 `sim/fake_robots/maps/`에 있다. `oneway_loop`는 이전 지도를 변환한 일방통행, `oneway_shortcut`은 거기에 양방향 지름길을 하나 넣은 것이다. `feature_map`은 11-06 결정 6의 특징 지도(넓은 길 2줄, 좁은 양방향 통로 `TUN`과 돌아가는 길, 지점 없는 교차 하나, 구역 14개)이고 우회 규칙을 검사한다. `two_lane_avenue`, `two_lane_segments`는 11-06 변형 지도로 레인 간격이 0.2 m라 안전한 배치가 아니고, 비교용으로만 둔다. 격자 지도는 시험 자료(`pc/fleet_core/test/data/grid_cells.yaml`)로만 남았다.
- 전수 검사(`tools/model_check.py`, 실제 `TrafficController`를 호출 순서마다 돌림)의 결과는 11절에 있다.
- 약점: 대로 전체가 잠금 하나이므로 대로에서 멈춘 로봇 1대는 그 이동을 모두 막는다.

## 4. 인터페이스 (`interfaces/fleet_interfaces`, FMS 통합 구조 기준)
계층: 사용자 → central → FMS → 핑키 / workcell(arm1·arm2) / arm3. central은 로봇을 직접 움직이지 않고, 로봇은 central을 모른다. 핑키 파트는 팔을 모르고 팔 파트는 핑키를 모른다.

**FMS 안의 연결은 ROS가 아니라 파이썬 호출이다.** `orders`와 `traffic`은 한 프로세스(노드 `fleet_manager`)의 두 모듈이므로 둘 사이에 액션, 서비스, 토픽을 두지 않는다. 합치기 전 판에 있던 `/fleet/dispatch` 액션, `/fleet/release` 서비스, `/fleet/events`, `/orchestrator/assignments`, `/orchestrator/zones`, `/orchestrator/{arm}/*`, `/orchestrator/confirm_clear`는 만들지 않는다. `Dispatch.action`과 `Release.srv`도 만들지 않는다.

FMS 내부 호출 (파이썬, `fleet_core` 패키지):
- `on_dispatch(goal_id, station_id, robot_id="")`: orchestrator → traffic. 로봇을 스테이션으로 보내 달라는 요청. `robot_id`가 비면 traffic이 고른다.
- `on_dispatch_cancel(goal_id)`: orchestrator → traffic. 아직 끝나지 않은 요청을 취소한다.
- `on_release(robot)`: orchestrator → traffic. 스테이션에서 할 일이 끝난 로봇을 놓아준다.
- `DispatchFeedback(goal_id, robot_id, phase)`, `DispatchResult(goal_id, success, robot_id, reason, error_x_cm, error_y_cm, error_yaw_deg)`: traffic → orchestrator. 반대 방향 보고. `phase`는 `NO_ROBOT` 하나뿐이다. 합치기 전 판의 CALLED, EN_ROUTE, QUEUED, DOCKING은 읽는 곳이 없어 만들지 않았다. 결과가 success면 그 로봇은 도크에 서서(AT_STATION) `on_dispatch(robot_id 지정)`나 `on_release`를 기다린다. 실패 이유 `reason`: `NOT_AVAILABLE`(쓸 수 없는 로봇), `BUSY`(이미 다른 요청이 있음), `CANCELED`, `ROBOT_REMOVED`, `REJECTED`(이동이나 도킹 목표 거절이 재시도 한도까지 이어짐), `DOCK_FAILED`(도킹 실패가 재시도 한도까지 이어짐). 그 밖에는 고정된 값이 아니다. 이동 실패가 재시도 한도에 닿으면 이동 결과의 `reason`을, `on_robot_fault`로 ERROR가 되면 그때 받은 이유를 그대로 넣는다.

두 모듈 사이는 위가 전부이고, `traffic`은 `orders`를 import하지 않으며 주문과 바구니 타입을 쓰지 않는다(경계 시험 `test_fms_boundary.py`). 다만 다음 세 가지는 이 호출로 표현할 수 없어 둘을 묶는 `FleetManager`가 따로 전달한다. 지시서의 "네 호출"에 없는 항목이므로 팀에 알린다.
- `on_run_mode(mode)`: central의 STOP/PAUSE/RUN을 두 모듈에 각각 넘긴다. 주문과 상관없이 달리던 로봇도 멈춰야 하므로 dispatch 취소로는 안 된다.
- `on_robot_removed(robot)`: `confirm_clear`가 받아들여지면 orchestrator에 알린다. 담기나 교환 중이면 돌려줄 dispatch가 이미 끝나 있기 때문이다.
- `reinstate(robot)`와 `on_robot_reinstated(robot, basket)`: `/fleet/reinstate`가 받아들여지면 traffic과 orchestrator에 각각 알린다(11-06 6번). 바구니는 orders만 안다.
- `on_robot_fault(robot, reason)`: 팔 재시도가 소진되어 orchestrator가 ERROR로 판단하면 traffic에 알린다. 통신 두절 판단과 호출 후보, "남은 로봇 없음" 판단이 이 상태를 본다.

바깥 인터페이스 (`contract.yaml` 0.3-draft와 같다. 11-05 구조 변경으로 기존 항목의 이름, 타입, QoS는 바뀌지 않았고, 더한 것은 11-05 지시서 4, 5번의 `/fleet/robots`와 heartbeat 둘이다. 11-06에 heartbeat 토픽만 `/fms/heartbeat`에서 `/fleet/heartbeat`로 이름이 바뀌었고 필드와 QoS는 같다):
- `/central/submit_order` SubmitOrder 서비스: UI → central. 재고 기록에 없거나 용량 초과면 거절
- `/central/set_run_mode` 서비스: UI → central (RUN / PAUSE / STOP)
- `/central/orders` OrderBoard 토픽(run_mode, 최종 보고가 안 된 주문을 오래된 순서로): central → FMS
- `/central/report_order` ReportOrder 서비스(SHIPPED, PARTIAL, HELD, placed, missing, detail): FMS → central. FMS는 `accepted=true` 답을 받을 때까지 답이 없는 보고를 `report_retry_s`마다 다시 보낸다. **실제 central이 지킬 것:** 같은 보고가 여러 번 와도 결과가 달라지면 안 된다. 이미 처리한 주문이면 재고를 다시 빼지 않고 `accepted=true`로 답한다(`fake_central_node`가 이렇게 한다). `accepted=false`는 다시 보내도 소용없을 때만 쓴다. FMS는 그 보고를 버리고 `ORDER_REPORT_REFUSED`를 낸다
- `/central/events` FleetEvent 토픽: FMS와 central이 발행하고 source로 구분한다
- `/fleet/assignments` AssignmentArray 토픽(항목마다 robot_id, order_id, station_id, task_type: TO_WORK/FILL/TO_OUTBOUND/SWAP/WAIT): FMS → central
- `/fleet/zones` ZoneStateArray 토픽: FMS → central
- `/fleet/confirm_clear` ConfirmClear 서비스: central(관리자) → FMS. 응답의 `string reason`은 11-06에 더한 관제 요구(팀 합의 전, 0.3-draft)다. 거부하면 이유, 받아들이면 빈 값. `accepted`, `released_zones`는 그대로다
- `/fleet/reinstate` Reinstate 서비스(robot_id, basket, note → accepted, reason): central(관리자) → FMS. 11-06에 더한 관제 요구(팀 합의 전, 0.3-draft)다. `confirm_clear`로 치운 로봇을 다시 넣는다. basket은 `BASKET_EMPTY`("EMPTY") 또는 `BASKET_NONE`("NONE"), 관리자가 본 트레이 상태다. 거부하면 `reason`에 이유, 받아들이면 빈 값. 거부 이유: 모르는 로봇, `OUT_OF_SERVICE`가 아닌 로봇, 쥔 구역이 남은 로봇, 틀린 basket(`INPUT_REJECTED`). 받아들이면 `ROBOT_REINSTATED`. 다른 로봇은 멈추지 않는다(배치 충돌 때 그 구역 주인만 예외). 배치는 시작 때와 같은 규칙(placed=false, 다음 위치 보고부터 시도하고 이동·도킹 중이면 기다림)이고, 배치 충돌로 세운 구역 주인은 우회 판단에서 무한 대기로 본다(5절)
- `/{pinky}/state` 토픽, `/{pinky}/move_to`·`/{pinky}/dock` 액션: FMS ↔ 핑키. `move_to`는 잠금을 쥔 데까지의 짧은 목표이고, goal의 `path`(geometry_msgs/Pose[])에 레인 폴리라인을 싣는다(5절, 11-06 지시서 7번). 11-06 지시서 4번에서 `state`에 `BLOCKED`, `LOCALIZING`을, `MoveTo` 결과에 `REASON_HEARTBEAT_LOST`를 더했다. 4번은 상수만 더해 필드가 그대로이고, `path`는 7번에서 더한 goal 필드다. 핑키가 지킬 동작은 `docs/핑키_계약.md`에 관제가 요구하는 것으로 적었다
  - `path` 요구(핑키 계약 3절 7번): `path`가 있으면 Nav2 FollowPath로 그 선을 따르고, 선 밖으로 돌아가거나 Spin·BackUp 같은 복구를 쓰지 않는다. 가장 가까운 점부터 따른다. 끝 근처 재전송과 도착 판정은 핑키가 한다. 목표가 없으면 움직이지 않는다. `path`가 비면 `target_pose`로 간다(`lane_goal_mode: point`)
  - `BLOCKED` 요구(핑키 계약 4절): `move_to` 실행 중, 1초 넘게 0.02 m/s 미만으로 서 있고, 그 이유가 경로 위 0.5 m 안의 물체일 때. 서고 2초 안에 보고하고, 막힌 동안 목표를 스스로 끝내지 않으며, 취소 뒤에도 물체가 있으면 계속 보고하고, 물체가 1초 안 보이면 끈다. 물체가 아닌 이유로 서면 `NAV_ABORTED`. 관제 처리는 5절 "레인 막힘"
- `/workcell/fill_basket` FillBasket 액션(items → placed, missing, success, reason): FMS → workcell. 팔 2대 배분과 조율은 workcell 내부 책임
- `/arm3/swap_basket` SwapBasket 액션(take_full, put_empty → took_full, put_empty_done): FMS → arm3
- 목표가 끝난 방식: `MoveTo`, `FillBasket`, `SwapBasket`의 결과에는 `REASON_CANCELED`("CANCELED")가 있다. 서버가 reason을 비워 두면 FMS가 목표 상태로 채운다. `STATUS_CANCELED`는 `CANCELED`, `STATUS_ABORTED`는 `ABORTED`다. 서버가 쓴 reason은 그대로 둔다. `FillBasket`, `SwapBasket`의 이 상수는 2026-11-06에 더한 관제 요구(팀 합의 전)다. 값은 README에 적힌 약속 그대로이고, 형식 설명에 상수가 들어가지 않으므로 직렬화와 타입 해시는 바뀌지 않는다
- `/{arm}/state` ArmState 토픽, `/{arm}/vision/items` ItemArray 토픽(arm1, arm2): 팔 파트가 발행한다. FMS의 판단에는 쓰지 않는다
- `/{pinky}/navigate_to_pose` 액션(Nav2): 핑키 안에서 pinky_fsm → Nav2. contract.yaml에 `real_only`로 있고, FMS는 부르지 않는다. `/{pinky}/follow_path`(nav2_msgs/action/FollowPath)도 `real_only`로 있다. `path` 목표를 따를 때 핑키 안에서 쓰고 FMS는 부르지 않는다
- `/cmd_vel`(Nav2, pinky_dock → pinky_bringup), `/scan`, `/odom`(pinky_bringup → Nav2): 핑키 안의 연결이다. 제조사 패키지와 Nav2의 일반적인 이름이고 contract.yaml에는 없다. 3대를 namespace로 나누는 구성은 시험 전이다

새로 더한 바깥 인터페이스 (팀 합의 전. 코드와 `contract.yaml`에는 들어 있다. `/fleet/robots`는 11-05 제안, `/fleet/heartbeat`는 관제 요구):
- `/fleet/robots` RobotStatusArray 토픽(LATCHED, 1 Hz): FMS → central. 로봇마다 robot_id, fsm_state(핑키 보고 그대로), task_type, order_id, fault(NONE, ERROR, COMM_LOST, OUT_OF_SERVICE, RECOVERING), fault_reason, wait_zone, wait_robots, battery, x, y, yaw. `/fleet/assignments`와 `/fleet/zones`는 호환을 위해 유지한다. 위치, 배터리, fsm_state는 마지막 보고 값이다
- `/fleet/heartbeat` std_msgs/Header 토픽(HEARTBEAT QoS: reliable, volatile, depth 1. 0.4-draft에서 STATE에서 바꿨다(지시서 1.5, 제안, FSM 담당 확인 전). 2 Hz, `heartbeat_period_s` 0.5 s): FMS → 핑키. 토픽과 핑키 쪽 동작 모두 관제가 요구하는 것이다(팀 합의 전, 핑키 계약 3절 5번). **핑키에 요구하는 것:**
  - 이동 또는 도킹 목표를 실행 중일 때, 마지막 heartbeat 또는 목표 수락에서 `heartbeat_timeout_s`(1.5 s, heartbeat 3번)가 지나면 그 목표를 취소하고 정지한다. move_to는 `reason=HEARTBEAT_LOST`로 끝낸다(11-06). 목표 수락도 FMS가 살아 있다는 증거로 친다. 새 FMS의 첫 목표가 첫 heartbeat보다 먼저 올 수 있기 때문이다.
  - 할 일이 없는 로봇은 heartbeat가 없어도 아무것도 하지 않고 상태 보고를 계속한다. 멈춘 뒤 온 새 목표는 받는다.
  - 시간은 벽시계로 잰다. 프로세스를 감시하는 값이기 때문이다.
  - 통신 두절도 같다. FMS가 살아 있어도 heartbeat를 못 받으면 멈춘다.
  - **FMS 쪽 약속:** 시작한 뒤 `heartbeat_timeout_s + heartbeat_period_s`(2 s) 동안 heartbeat를 내지 않고 상태 보고도 받지 않는다. 이전 FMS의 목표로 달리던 로봇은 보통 그 사이에 멈춘다. 늦게 수락된 목표나 감속 중인 로봇은 아직 움직일 수 있으므로 침묵만 믿지 않는다. 그 뒤 로봇의 `move_to`, `dock` 서버를 찾았고, 전원이 위치를 보고했고, `MOVING`, `DOCKING`, `BLOCKED`, `LOCALIZING`이 아니어야 지도에 올리고 움직이기 시작한다. heartbeat도 전원이 지도에 오른 뒤에야 낸다. 그때까지 이 FMS가 보낸 목표가 없으니 필요가 없고, 늦게 수락된 이전 목표가 새 heartbeat를 자기 것으로 알고 계속 달리는 일을 막는다.
  - **이 계약이 보장하지 않는 것:** 마지막 heartbeat 뒤 도착한 목표는 수락 시각부터 다시 `heartbeat_timeout_s`를 센다. 이전 FMS의 목표가 로봇이 지도에 오른 뒤에 도착하면 그 로봇은 `heartbeat_timeout_s`까지 움직이고, FMS는 그 사이 옛 위치를 믿는다. 0.4부터 heartbeat는 reliable(depth 1)이라 잃은 것을 다시 보내지만, 끊김 판단은 앱 타이머(벽시계)라 링크가 1.5 s 넘게 끊기면 FMS가 살아 있어도 멈춘다. DDS `deadline`·`liveliness`는 쓰지 않는다. `/{pinky}/state`는 best_effort 그대로이고 주기적으로 계속 와야 한다. 멈춘 로봇의 감속 거리는 timeout에 들어 있지 않다.

run_mode는 FMS가 OrderBoard로 받는다. 삭제된 것: TaskBoard, StationCount, BoxArray, ArmTransfer, ReportDone, `/{arm}/load`, `/{arm}/unload`, `/central/tasks`, `/central/report_done`.

## 5. 교통 제어 규칙 (`traffic`, 핑키만)
구역과 충돌쌍은 3절. 유지: 구역 잠금(현재+다음 1개), 구역별 FIFO, 위치 3회 확인 후 해제, 교착 감지, 통신 두절/복구, 이동 재시도, 시작 시 전원 보고 대기.

이동 명령(레인 준수, FollowPath): 관제가 잠금을 쥔 데까지의 짧은 `move_to`를 보낸다. 이어지는 레인은 방향이 바뀌어도 잠금을 쥔 데까지 한 목표로 묶고, goal의 `path`에 그 레인 폴리라인을 싣는다. 양방향 레인에서 되돌아가는 유턴만 끊는다. 유턴을 이으면 경로를 바꿀 때마다(호출, 취소) 목표 끝이 앞으로 밀려 경로가 끝없이 늘어난다(전수 검사에서 찾음). 첫 점은 로봇이 보고한 위치, 이어서 아직 지나지 않은 지점, 끝 점은 목표 지점(`target_pose`)이다. 지점을 빼는 것은 로봇이 나가는 레인을 따라 `arrival_tolerance_m`(0.15 m) 넘게 갔고 들어오는 레인보다 나가는 레인에 가까울 때뿐이다. 그래서 모서리 앞에서 위치 잡음으로 모서리를 가로지르지 않고, 0.15 m 안으로 지난 지점이 선 앞에 남을 수 있다. 그 앞부분은 핑키가 건너뛴다(FollowPath는 가장 가까운 점부터 따른다. FakePinky와 SimWorld는 이미 지나 다음 선분 위에 있는 앞 점을 건너뛴다). 각 점의 방향은 다음 점을 향하고 끝 점은 마지막 레인 방향이다. 핑키는 Nav2 FollowPath로 그 선을 따른다. 잠금이 더 오면 관제가 더 긴 선으로 새 목표를 보낸다(선점). 새 목표를 보낸 뒤 온 이전 목표의 결과는 이유가 무엇이든 버린다. 늘림과 중단이 겹치면 중단 한 번이 재시도로 세어지지 않지만, 원인이 남으면 새 목표도 실패해 그때 센다(11-06 `shared_lane_failure`에서 확인). 끝 근처 재전송과 도착 판정은 핑키 몫이다(핑키 계약 3절 7번). 관제는 지금처럼 성공 결과와 위치로 도착을 확인한다(아래 "도착 판정"). `lane_goal_mode: point`면 예전처럼 같은 방향 레인만 묶고 방향이 바뀌면 끊으며 `path`는 비운다. 호환과 비교용이다. **이 방식은 11-06에 FollowPath로 정했고(13절) FSM·Nav2 담당과 확인 전이다.**

경로 연결 확인(11-07 지시서 7.5, 제안): 핑키는 받은 `path`가 제 자리에서 `lane_error_warn_m`(0.15 m)보다 멀면 따라가지 않고 `PATH_NOT_CONNECTED`로 끝낸다(핑키 계약 3절 9번). 이전 목표는 새 목표를 받을 때 끝나므로 로봇은 선다. 관제는 다른 이동 실패처럼 실패 한 번으로 세고, `move_retry_wait_s` 뒤 지금 보고 위치에서 path를 다시 만든다. 이유는 `GOAL_RETRY` detail에 남는다. 관제 코드는 바꾸지 않았다. 첫 점이 보고 위치이고, 보고 위치는 로봇과 같은 자리거나 로봇이 지나온 path 위에 있으므로(보고 지연) 위치 잡음이 0.15 m보다 훨씬 작으면 정상 운행에서는 이 실패가 나지 않는다. 마지막 보고 뒤 위치 추정이 튀면 거절될 수 있다(시뮬에서는 drift가 막 시작된 순간. 그래서 `off_lane` 시나리오는 이 수를 세지 않는다). 그때는 재시도가 새 보고로 path를 다시 만든다. SimWorld는 보고하고 같은 step에 명령을 받는다. 그래서 로봇이 거절할 만큼 먼 path가 주입한 것도 아니고 관제가 받아들인 마지막 위치도 지나지 않으면 불변식 위반으로 본다(Codex 검수). 관제가 버린 보고(지도 밖)는 그 위치를 바꾸지 않는다. 그 위치를 지나는데도 로봇에서 멀면 원인을 가리지 않고 핑키처럼 거절로 센다(예: 한계 없는 잡음 `gaussian_noise`가 보고를 튀게 한 경우). 지시서는 "path는 마지막으로 지난 지점에서 시작한다"고 적었지만, 7번에서 정한 대로 첫 점은 보고 위치로 둔다. 둘 다 로봇이 path 위에 있게 하는 같은 목적이고, 핑키 쪽 구현(`pinky_fsm`)이 이미 이 계약을 따른다. FakePinky와 SimWorld의 로봇도 같은 검사를 한다(path가 비면 재지 않는다). 시험은 `test_path_connection.py`(떨어진 path를 한 번 받으면 실패 1회, 재시도, 출고. 정상 운행은 지도 셋, 두 모드에서 0회)와 FakePinky의 ROS 시험이다.

꺾임을 넘는 목표와 다른 규칙(`traffic/lane_path.py`, `test_lane_path.py`): 경로 인덱스는 레인마다 d를 지날 때 넘어가므로 목표가 꺾임 너머에 있어도 해제(d), 지점 도착 판정(목표 지점까지의 거리), 멈춤 감시(위치), 레인 막힘의 "달리던 레인"(`_running_lane`, 경로 인덱스 앞뒤 레인 중 가까운 쪽)은 그대로 맞다. 바꾼 것은 이탈 e 하나다. point는 목표 안에서 시작점을 지난 마지막 레인(없으면 첫 레인)으로 재지만 path는 이 보고로 넘길 진행 인덱스 앞뒤 두 레인 중 가까운 쪽으로 잰다(아래 이탈 항목). 꺾인 목표의 첫 레인을 달리는 로봇이 마지막 레인 기준으로 멀다고 `LANE_ERROR`가 나지 않게 하려는 것이다.

잠금 규칙:
- 로봇은 현재 구역 말고는 쥔 것이 없을 때만 다음 구역을 청한다. 그래서 대기하는 구역은 많아야 하나다(`wait_zone`).
- 구역마다 FIFO다. 충돌쌍에는 FIFO를 넓혀 쌍 전체의 요청 순서를 따른다. 한쪽이 계속 바빠도 다른 쪽이 굶지 않게 하려는 것이고, 지시서의 "레인별 FIFO"보다 넓다. 이 순서로 대기 고리가 생기는지는 지도마다 전수 검사로 본다(11절).
- 해제: 로봇 위치 p를 그 구역을 떠나는 레인(시작 지점 p_i, 단위 벡터 u)에 투영해 s = (p − p_i)·u를 구한다. s > d(`release_distance_m`)가 `release_confirm_samples`번 연속이면 해제한다. 떠나는 레인이 경로에 없으면(앞쪽 구역, 손으로 옮긴 로봇) 구역 중심선에서 d 이상이어야 한다. `release_clear_of_floor`가 켜져 있으면(`fleet_params.yaml` true, 데이터클래스 기본값은 false) 경로 위 해제에도 그 구역의 다른 바닥 선(나가는 레인의 역방향은 뺀다)에서 중심선 거리 d 이상을 더한다. 근거식 d > 2r + ε는 구역의 레인들이 90도 넘게 벌어질 때만 맞다. 나가는 레인이 다른 레인과 예각 θ로 만나면 s만큼 가도 그 레인과 s·sinθ만 떨어지므로 그 거리도 봐야 한다(특징 지도 T1: θ = 53.1도, s = 0.2에서 0.16 = 2r, 잡음이 더해져 0.148). 시작 때 구역을 떠나는 모든 레인의 끝 지점이 그 기준을 넘는지 보고 어기면 시작을 거부한다. 로봇이 지점에 선다고 가정한다. `arrival_tolerance_m`만큼 못 미쳐 서면 안 놓일 수 있다(사용자 결정 대기, 13절). 기본 지도에서는 S→L(N→S→L이 90도 넘게 꺾임)에서 해제가 약 1 cm(s 0.21) 늦어진다.
- 근거식: d ≥ r + ε + v·(τ + tick), d ≥ 2r + ε. r은 반경, ε은 위치 오차, v는 속도, τ는 보고 지연, tick은 관제 주기다. 뒤의 식은 앞 로봇이 해제 지점 근처에 서고 뒤 로봇이 지점까지 오는 경우이고, 시험으로 만들지는 않았다. d ≤ 2r이면 `TrafficController`가 시작을 거부한다. d는 실측 전 값이다. 압박 시험 표는 작업 기록 7번에 있고, 실측 ε과 τ가 나오면 그 표로 d를 정한다.
- 이탈 e = |(p − p_i) − s·u|는 해제 조건이 아니다. 대신 이탈 중에는 해제를 멈춘다(13절, 11-06 지시서 3번). e는 이동 목표 안에서 로봇이 지금 달리는 레인(시작점을 지난 마지막 레인)으로 잰다. `path` 목표는 꺾임을 넘으므로 진행 인덱스(이 보고로 넘길 곳까지 셈한다) 앞뒤 두 레인 중 가까운 쪽으로 잰다. 진행이 첫 레인에 있는 동안(첫 레인을 해제 거리만큼 지나 꺾임을 돌기 전)은 첫 지점으로 들어오는 레인 중 가까운 것(두 번째 지점에서 오는 레인은 뺀다)도 후보이고, 경로 인덱스를 넘길 때와 첫 지점을 선에서 뺄 때도 이 레인과 비교한다(`lane_path.py`의 `_lane_before`, e85edff에서 예전의 첫 지점 앞 면제를 없앴다). 후보는 보고마다 다시 고른다. 그래서 다른 진입 레인 위로 밀리면 이탈이 가려질 수 있고, 후보가 바뀌면 횟수가 0으로 돌아간다(후속 과제). 꺾임 앞에서 옆으로 밀린 보고나 예각에 다가오는 로봇이 다음 레인에 앞서 걸리지 않게 한다. 횟수는 그 레인이 바뀔 때만 0으로 돌아가고, 목표를 연장하거나 다시 보내도 이어진다. 경로의 첫 지점으로 가는 목표와 도킹은 기준이 될 레인이 없어 보지 않는다. 목표가 없을 때 온 보고와 `LOCALIZING` 보고는 세지도 끊지도 않는다. 그래서 "이어지면"은 이탈을 잰 보고끼리 이어진다는 뜻이다.
- `lane_error_warn_m`을 넘는 보고가 `release_confirm_samples`번 이어지면 `LANE_ERROR`(WARN)를 내고 목표를 취소한다(`CancelCmd`). 쥔 구역은 놓지 않고, 기다리던 구역 요청은 거둔다(뒤 로봇을 막지 않게). 그동안 해제 판단(거리 해제, 통신 복구와 교착 후퇴의 해제)을 멈추고, 새 목표를 내지 않고, 배차와 호출에서 빠지며, 목표가 없으니 멈춤 감시도 쉰다. `/fleet/robots`에는 `fault=RECOVERING`, `fault_reason=OFF_LANE`으로 보인다. 벗어난 로봇의 실제 위치를 모르는데 구역을 놓으면 다른 로봇이 그 공간에 들어올 수 있기 때문이다.
- 보고가 그 레인 구간(양 끝에서 `half_width_m`까지 포함)에서 `half_width_m` 안으로 `release_confirm_samples`번 이어지면 `BACK_ON_LANE`(INFO)을 내고 같은 지점으로 목표를 다시 보낸다. 시도 횟수로 세지 않는다. 서 있던 동안의 보고로는 구역을 놓지 않는다(벗어난 보고 수를 지운다). `off_lane_timeout_s` 안에 못 돌아오면 `OFF_LANE`(ERROR, `fault_reason=OFF_LANE`)이고 그 뒤는 관리자가 정한다(`confirm_clear`나 재투입). 시간이 지난 뒤 찬 복귀 보고는 시간 초과를 덮지 않는다. 마감 전에 복귀 횟수가 찼으면 통신 복구를 기다리느라 마감을 넘겨도 시간 초과가 아니다. 다시 벗어나 횟수가 0이 되면 마감이 다시 걸린다. 서 있는 동안 출고 배차가 취소된 `TO_OUTBOUND` 로봇은 복귀 뒤 같은 목표 대신 대기 지점으로 간다(`off_lane.py`의 `_watch_off_lane`).
- 이탈 상태는 통신 상태(`COMM_LOST`, `RECOVERING`)와 따로 둔다. 이탈 중 통신이 끊기면 `COMM_LOST`가 보이고, 보고가 돌아와도 레인에 돌아올 때까지 통신 복구의 위치 확인을 미룬다(`RECOVERING`, CHECKING_POSE). 레인에 돌아오면 통신 복구가 먼저 끝나고 그 뒤 이탈을 푼다. 이때 경로는 통신 복구 규칙대로 쥔 구역 안의 지점부터 다시 잡으므로 원래 목표와 다를 수 있다. 이탈 시계는 통신이 끊긴 동안에도 간다. 다른 고장(ERROR)이나 `confirm_clear`가 오면 이탈 상태는 지운다.
- 가장 짧은 레인이 d 이하이면 레인 끝에 서도 해제되지 않으므로 `TrafficController`가 시작을 거부한다. 기본 지도의 가장 짧은 레인은 0.4 m다.

시작과 재시작: 전원이 지도에 올라야 움직이기 시작한다. 로봇은 위치를 보고하고 `MOVING`, `DOCKING`, `BLOCKED`, `LOCALIZING`이 아닐 때 지도에 오른다. `LOCALIZING` 보고는 자세를 쓰지 않고 지도 밖 검사도 하지 않는다. 지도에 오른 로봇이 `LOCALIZING`만 보내면 보고 시각을 갱신하지 않아 `comm_timeout_s` 뒤 `COMM_LOST`가 되고, 구역은 쥔 채 둔다. 위치를 다시 보고하면 복구 확인 뒤 이어 간다. 복구 확인 중 마지막 보고가 `LOCALIZING`이면 복구를 끝내지 않고, 위치 보고가 다시 오면 그때부터 확인을 잰다. `startup_wait_s`가 지나도 안 오른 로봇이 있으면 `ROBOT_NOT_REPORTING`을 낸다. 아래의 `PLACEMENT_CONFLICT`로 거부된 로봇은 이미 보고했으므로 여기서 세지 않는다. 다른 로봇이 쥔 구역이나 그 충돌쌍에 서 있으면 지도에 올리지 않고 `PLACEMENT_CONFLICT`를 내며, 그 로봇이 움직이거나 관리자가 치울 때까지 **전원이 기다린다**(정책, 팀 확인 필요). 지점에서 d보다 멀리 서 있으면 먼저 그 레인의 끝으로 간다. 전원이 오르기 전에는 `/fleet/zones`를 내지 않는다. FMS는 잠금을 아직 모르며, 모르는 것과 빈 것은 다르다.

교착 처리: 대기 고리가 있는 동안 새 호출과 새 이동 단계를 내지 않는다. 이미 보낸 이동 목표는 그대로 두고, 아래의 포기 대상 로봇만 목표를 취소한다. 고리가 생기고 `deadlock_grace_s`가 지나면 우선순위가 가장 낮은 로봇 1대에서 아직 들어가지 않은 구역을 포기 대상으로 정하고, 대상이 있으면 이동 목표를 취소한다(`DEADLOCK_RETREAT`). 실제 해제는 `recover_check_s` 뒤에 위치 보고로 벗어난 것이 확인된 구역만 한다. 들어간 구역에서 물러나는 동작은 없다. 포기할 구역이 없으면 바로 `DEADLOCK_UNRESOLVED`로 알린다. 재확인 때 구역이 하나라도 해제되면 이 이벤트는 나지 않는다. 아무것도 해제하지 못했어도 대상 중 하나라도 해제 여유 밖에 있으면 재확인을 반복하며, 이 대기에는 시간 제한이 없다. 아무것도 해제하지 못했고 모든 대상의 해제 여유 안에 들어가 있으면 `DEADLOCK_UNRESOLVED`로 알린다(7절, SR-13과 다른 점은 12절).
우선순위 입력: 스테이션에 서 있는지(AT_STATION)와 출고 구역으로 가는 요청을 받았는지만 본다. 주문과 바구니는 보지 않는다.
orchestrator로 옮긴 것(5-1절): 주문 묶기, FILLING/SWAPPING, 팔 재시도·시간 초과, 바구니 상태, ReportOrderCmd.
삭제: 양쪽 물량 비율 배정(Rebalancer의 2:1, flip), 상자 개수, 비전 개수로 적재량 추정(`_moved_by_vision`).

호출(dispatch 처리):
- 로봇 이름 없는 `on_dispatch(goal, WORK)`는 열린 호출로 쌓인다. traffic은 매 주기 열린 호출 수만큼 로봇을 고른다. 후보는 할당이 없는 사용 가능 로봇(AT_STATION이면 `on_release`를 받은 로봇)이고, 점수가 가장 낮은 로봇을 고른다. 점수는 담기 도크까지 레인을 따른 거리에 `score_distance_weight`를 곱한 값이고, 이미 호출된 로봇이면 `score_same_call`을 더한다. 대기 지점에서는 앞 로봇을 추월할 수 없으므로 앞 로봇부터 호출한다. 고장 난 로봇 뒤의 대기 로봇은 호출하지 않는다. 호출은 주기마다 다시 계산하므로 다른 로봇이 이 보정을 넘을 만큼 가까워지면 호출이 옮겨 간다. 고를 로봇이 없으면 호출을 연 채 기다린다. 지도에 올라 고장이 아닌 로봇이 하나도 없을 때만 `DispatchFeedback(phase=NO_ROBOT)`을 보낸다(멈춤 한 번에 한 번). 전원이 통신 두절인 경우는 돌아올 수 있으므로 보내지 않는다. orchestrator가 이것을 받아 `NO_ROBOT_AVAILABLE`(WARN)을 낸다.
- 호출이 줄었는데(`on_dispatch_cancel`) 그 로봇이 스테이션 `commit` 지점(기본 지도는 W)의 구역을 쥐기 전이면 대기 구간으로 돌린다. 쥐었으면 끝까지 가서 도킹 없이 대기 구간으로 간다.
- 도킹을 마치면 `DispatchResult`(success, robot_id, 도킹 오차)를 돌려주고 로봇은 AT_STATION이 된다. 다음 `on_dispatch`(robot_id 지정)나 `on_release`가 올 때까지 도크를 비우지 않는다.
- `on_dispatch(goal, OUTBOUND, robot_id=X)`는 지정한 로봇을 보낸다. `on_release(X)`를 받은 로봇은 다음 주기의 호출 후보가 된다. 뽑히면 바로 작업공간으로 가고(작업 도크에 서 있었으면 그 자리에서 결과를 돌려준다), 안 뽑히면 대기 구간으로 간다. 합치기 전 판의 "밀린 주문이면 orchestrator가 그 로봇을 지정해 다시 부른다"는 쓰지 않는다. 대기 지점에 더 가까운 로봇이 있어도 먼 로봇을 부르게 되어 지금 동작과 달라지기 때문이다.

임무 단계: IDLE, PARKED, TO_PARK, TO_WORK, TO_OUTBOUND, DOCKING, AT_STATION (고장 표시: ERROR, OUT_OF_SERVICE, COMM_LOST, RECOVERING). 바구니 상태와 주문은 traffic이 갖지 않는다.

STOP: 이동 목표를 취소한다. 열린 dispatch는 그대로 두고 RUN이면 이어서 간다. PAUSE: 진행 중인 이동은 끝내고 새 이동을 내지 않는다.

confirm_clear 거부(`accepted=false`, `reason`에 이유, 아무것도 바꾸지 않음): 없는 로봇이나 목록에 없는 구역 이름이 하나라도 있으면 `INPUT_REJECTED`, 구역을 쥔 로봇인데 목록이 비었으면 `CLEAR_REFUSED`. 구역을 쥐지 않은 로봇의 빈 목록은 정상이다. 쥔 구역의 일부만 보내면 관리자의 의도로 보고 받아들이되 `ZONES_STILL_HELD`로 알리고, 다시 호출하면 나머지가 풀린다. OUT_OF_SERVICE 로봇의 구역은 위치로 풀리지 않기 때문이다.

자동 격리(11-06 개선 지시서 5.5번, `traffic/isolation.py`): 지도에 오른 로봇이 ERROR가 되는 순간 플릿이 한 번 격리한다. 로봇은 서서 보고하고, 복구는 플릿이 한다. 로봇 1대 고장 시 플릿은 계속 운행하고(fail-safe but not fail-stop), 사람은 치우는 것만 한다.
- 거는 곳: `_fail`을 지나는 모든 전이. 이동·도킹 재시도 초과(`GOAL_FAILED`, 멈춤 감시의 `STALLED`와 `NOT_AT_TARGET` 포함), 막힘 60초(`BLOCKED_TOO_LONG`), 복구 충돌(`RECOVERY_CONFLICT`, 이제 `_fail`을 지난다), 주문 쪽이 보낸 `RobotFaultCmd`(팔 재시도 초과 `FILL_FAILED`·`SWAP_FAILED`, 일부 담김 금지의 `FILL_FAILED`, `ARM_NO_RESULT`). 파트 3의 레인 이탈 시간 초과(`OFF_LANE`)도 `_fail`을 지나므로 격리되고 주문이 되돌아간다(`test_off_lane.py`).
- 걸지 않는 곳: `PLACEMENT_CONFLICT`. 지도에 오르지 않아 쥔 구역도 주문도 없고, 놓이면 스스로 풀린다. 이미 ERROR인 로봇에 다시 오는 `_fail`도 다시 격리하지 않는다.
- 하는 일: 구역은 그대로 쥔다(몸이 거기 있다). 구역 요청, 지정 배차(실패로 답함), 진행 중인 목표를 취소한다. `RobotIsolated(robot, reason)`를 주문 쪽에 넘기고 `ROBOT_ISOLATED`(ERROR)를 한 번 낸다. 주문 쪽은 그 로봇의 주문을 대기 맨 앞으로 되돌리고(`ORDER_REQUEUED`) 바구니를 NONE으로 둔다(5-1절).
- 다른 로봇은 그 구역을 피해 간다(파트 5 우회). 우회로가 없으면 기다린다. 기본 지도는 작업대와 출고대로 가는 길이 하나라, 도크나 대로에서 격리된 로봇 뒤의 로봇은 관리자가 치울 때까지 기다린다. 그 로봇보다 앞서 간 로봇은 계속 출고한다.
- `RobotIsolated`는 `RobotFaultCmd`(주문 → 교통)와 방향이 반대다(교통 → 주문). traffic이 commands.py에서 가져오는 이름에 하나 더했다(`test_fms_boundary.py`). 주문은 모른다.

confirm_clear(관리자가 로봇을 치웠다): 구역 해제, OUT_OF_SERVICE, 이동 목표 취소. 그 로봇을 지정한 dispatch가 진행 중이면 실패(reason=ROBOT_REMOVED)로 돌려준다. 격리된 로봇이면 주문은 대개 이미 돌아갔으므로 구역만 푼다. 예외 둘은 주문이 아직 묶여 있다: 격리 때 교환 중이라 취소 결과를 기다리는 주문, `ARM_NO_RESULT`로 늦은 결과나 `confirm_clear`까지 쥔 주문(5-1절). 이 둘과 격리되지 않은 로봇(사람이 먼저 치운 경우)이면 orchestrator가 주문도 되돌린다(`FleetManager`가 `on_robot_removed`를 부른다). 재투입(파트 6)은 OUT_OF_SERVICE 로봇만 받으므로, 격리된 로봇도 `confirm_clear` 뒤에야 다시 들어온다.

reinstate(관리자가 치운 로봇을 다시 넣는다, 11-06 6번, `traffic/reinstate.py`): `OUT_OF_SERVICE`인 로봇만 받는다. 쥔 구역이 남았으면 거부한다(위치로 풀리지 않는 구역이고, 올라온 뒤 셋을 쥘 수 있다. `confirm_clear`로 먼저 푼다). 받아들이면 그 로봇의 기록을 새로 만들어(placed=false, 단계·목표·재시도 0) 맵 밖에 둔다. 다음 위치 보고 때 시작과 같은 절차로 올린다: MOVING이나 DOCKING 보고면 기다리고, 그 자리 구역을 못 잡으면 `PLACEMENT_CONFLICT`. 시작 때와 다른 점: 올라오기 전에도 다른 로봇은 멈추지 않는다(`_not_placed`에서 뺀다). 이미 운행 중이고, 치운 동안에도 잠금 없이 서 있던 로봇이기 때문이다. `/fleet/zones`와 heartbeat도 계속 나간다. 다만 남이 쥔 구역(충돌쌍 포함)에 보고돼 `PLACEMENT_CONFLICT`가 나면 그 주인 로봇을 세운다: 진행 중 목표와 구역 요청(대기열 자리, `wait_zone`)을 취소하고, 새 목표·지정 배차(`NOT_AVAILABLE`)·열린 호출을 주지 않는다. 열린 호출은 다른 로봇에게 갈 수 있다. 이미 받은 지정 배차는 남겨 두고 풀린 뒤 이어 간다(구역도 그때 평소처럼 다시 청한다). 세운 주인에게 막힘이 열려 있으면 그 취소도 막힘 표시 때처럼 결과를 기다린다(`block_cancel`). 결과 전에는 구역을 놓지 않고 목표를 다시 보내지 않는다(병합 리뷰 1번). 세운 주인은 우회 판단에서 못 비키는(무한) 로봇이다. 저절로 풀리지 않기 때문이다. 자동 격리(5.5)된 로봇도 `confirm_clear` 뒤 같은 절차로 다시 넣는다. 격리 때 바구니는 NONE으로 두었으므로, 재투입 때 관리자가 본 값으로 바뀐다. 재투입 로봇이 맵에 오르거나 다시 치워지면 풀린다. **둘 다 제자리면 저절로 풀리지 않는다.** 관리자가 재투입 로봇을 빈 자리로 옮겨 다시 보고하게 하거나 다시 치워야 한다(11-06 리뷰 두 번). 올라오기 전에는 호출 후보가 아니고 일도 받지 않는다. orders는 그 로봇의 바구니를 요청 값으로 두고 주문 없음, 재시도 0으로 시작한다. basket=NONE이면 주문을 받기 전에 출고 구역으로 보내 빈 바구니를 받는다(SwapCmd take_full=false, put_empty=true). 잃은 팔 작업(치울 때 취소, 또는 `ARM_NO_RESULT`)의 결과가 늦게 오면, 그 로봇이 같은 주문을 다시 묶었어도 그 주문에 넣지 않고 `ARM_LATE_RESULT`로 알린다. 주문을 묶을 때마다 차례 번호를 매기고 잃은 작업에 남겨 비교한다.

이동과 도킹의 재시도: 목표가 실패하면(거절과 현재 목표의 `CANCELED` 결과도 실패다. `HEARTBEAT_LOST`는 아래처럼 세지 않는다) 시도 횟수를 하나 늘리고, `move_retry_wait_s`가 지날 때까지 그 로봇에 새 이동이나 도킹 목표를 보내지 않는다. 달리고 있는 목표를 연장하는 것은 막지 않는다. 기다리는 동안 구역은 쥐고 있다. 세 번째 시도까지 실패하면 ERROR(`GOAL_FAILED`)다.

멈춤 감시(11-06 개선 지시서 2번): 이동이나 도킹 목표가 열려 있는데 보고 위치가 `stall_s`(도킹은 `dock_stall_s`) 동안 `stall_move_m` 미만으로만 움직이면 `GOAL_STALLED`(WARN)를 내고 그 목표를 취소한다. 실패한 시도 하나로 세므로 `move_retry_wait_s` 뒤 같은 지점으로 다시 보내고, 세 번째 시도까지 멈추면 `GOAL_FAILED`(ERROR, `fault_reason=STALLED`)다. `GOAL_RETRY`는 따로 내지 않는다. 위치는 보고 그대로가 아니라 `stall_smooth_s` 시간 상수의 지수 평균으로 본다. 재는 시작점은 목표가 보인 첫 tick의 평균 위치이고, 평균이 그 점에서 `stall_move_m` 이상 움직이면 그 자리에서 다시 잰다. 시드 200개 시험에서 σ 0.03 m 잡음의 제자리 로봇은 모두 ERROR까지 갔고 0.2 m/s 주행은 한 번도 멈춤으로 잡히지 않았다. σ 0.05 m에서는 200개 중 20개가 30초 안에 ERROR까지 가지 못했다. 목표를 연장해도 시계는 그대로다. 목표가 없을 때(구역을 기다리며 서 있음, 스테이션에서 대기), RUN이 아닐 때(PAUSE, STOP), 통신이 끊기거나 복구 중이거나 고장일 때는 재지 않고 처음부터 다시 잰다. 취소한 목표의 결과는 목표 번호가 달라 무시된다. 전수 검사는 시계가 멈춰 있어 이 감시가 끼어들지 않는다.

레인 막힘(11-06 개선 지시서 4번, 핑키 계약 4절, `traffic/blocking.py`): 이동 목표 중 `BLOCKED` 보고가 오면 막힘을 연다. 레인은 그때 달리던 레인이다. 경로의 들어오는 레인과 나가는 레인 중 로봇에 가까운 쪽이고, 같으면 나가는 레인이다. 경로상 같은 레인이나 다음 레인 앞 `queue_gap_m` 안에 정상 로봇(지도에 있고 고장, 두절, `BLOCKED`, `LOCALIZING`, 막힘이 아님)이 있으면 줄 서기로 보고 열지 않는다. 목표가 없거나 통신이 비정상이면 상태만 보인다. 막힘을 연 동안과 줄 서기 동안은 멈춤 감시를 쉰다. 달리던 레인을 찾지 못한 `BLOCKED`는 막힘을 열지 않으므로 멈춤 감시가 잰다. 경로 레인은 거리 제한 없이 가까운 쪽을 고른다. 경로가 아닌 레인에 귀속시키지 않기 위해서다. 레인에서 멀리 벗어난 로봇은 `LANE_ERROR`가 맡는다(파트 3: 이동 목표가 열린 동안 세운다). 막힘 표시 전에 이탈로 서면 막힘은 닫고(`LANE_CLEARED` 없음) 이탈이 맡는다. 돌아와 목표를 다시 보낸 뒤 `blocked_rejoin_s` 안에 같은 레인에서 다시 막히면 막힌 시간을 이어 센다. 막힘을 표시한 뒤에는 목표가 취소되어 이탈을 재지 않으므로, 그동안 구역은 막힘 쪽 규칙(취소 결과, 선 것 확인, 앞 구역의 바닥 밖 보고)으로만 놓는다. `/fleet/robots`의 순서는 고장, 통신, 이탈(RECOVERING, OFF_LANE), 막힘(NONE, BLOCKED)이다.
- 시간은 tick마다 `BLOCKED`이고 통신이 정상이며 줄 서기가 아닐 때만 더한다. 두절과 복구 확인 동안, 막힘을 연 뒤 앞에 정상 로봇이 선 동안은 멈춘다. 줄 서기가 `blocked_timeout_s` 넘게 이어지면 더는 쉬지 않고 막힘으로 친다. 앞 로봇이 뒤 로봇의 구역을 기다려 서로 막고 서면 구역 대기 그래프에 보이지 않기 때문이다. 그때부터 막힘 시계로 `blocked_mark_s` 뒤에 표시하고 `blocked_timeout_s` 뒤에 ERROR다. 줄 서기 시간은 `BLOCKED`가 아닌 보고나 통신 두절이 끼면 처음부터 다시 잰다. tick(0.1초) 단위로 더하므로 상태가 바뀔 때마다 한 tick까지 틀릴 수 있다.
- `blocked_mark_s`: 레인을 표시하고(`LANE_BLOCKED`) 그 로봇의 잠금 요청과 목표를 취소한다. 몸이 있는 레인 양 끝의 구역은 쥔다. 나머지 쥔 구역은 취소한 목표의 결과가 오고 이동 중이 아닌 보고가 `release_confirm_samples`번 이어진 뒤, 바닥 밖이 확인된 것만 놓는다. 표시 동안 구역은 이 규칙으로만 놓고, 다른 자동 해제(지나간 구역, 복구, 교착 후퇴)는 쉰다. 취소한 목표의 결과가 오기 전에는 막힘이 먼저 풀려도 그 로봇의 어느 구역도 자동으로 놓지 않는다(`_release_if_cleared`, `block_cancel`). 결과가 끝내 오지 않으면 노드가 1초마다 취소를 다시 보내고 5번 뒤 `CANCEL_UNCONFIRMED`(WARN)를 낸다. 구역은 쥔 채 두고 관리자가 `confirm_clear`로 푼다. 정지는 상태 보고(`BLOCKED`는 계약상 1초 넘게 0.02 m/s 미만)와 취소 결과로 본다. 원시 자세로 재면 σ 0.03 m 잡음에 흔들려 넣지 않았다. 표시 동안과 취소한 목표의 결과가 오기 전에는 그 로봇의 단계를 진행하지 않는다(새 구역 요청, 이동, 도킹을 내지 않는다). 결과 전에 새 목표가 다시 막혀 표시되면 앞 취소의 추적을 잃기 때문이다. 그래서 이 로봇은 구역 대기 그래프에 들어가지 않고 교착 후퇴 대상도 되지 않는다. 이 동안 새 호출은 받지 않는다. 이미 맡은 호출은 다른 로봇처럼 선택기가 다시 고른다(같은 호출 가산점). 대기 줄에서는 앞 로봇으로 남아 뒤 로봇이 앞지르지 않는다. 지정 배차는 받아 두고 결과가 오면 이어 간다. 노드가 결과를 잃으면(결과 future 오류) 취소한 로봇 목표는 거절로 넘기지 않는다. `CANCEL_UNCONFIRMED`를 낸 뒤에는 그 취소를 다시 보내지 않고, 코어는 결과나 `confirm_clear`까지 구역을 쥔다.
- 풀림: 통신이 정상이고 `BLOCKED`가 아닌 보고가 `release_confirm_samples`번 이어진다. 두절이나 복구 확인 중의 보고는 풀림 셈과 정지 셈을 0으로 되돌린다. 표시했으면 `LANE_CLEARED`를 내고 구역을 다시 청해 같은 목표를 다시 보낸다(시도로 세지 않는다). 취소 결과보다 풀림이 먼저 오면 결과를 받은 뒤 다시 보낸다. 그동안 선택기가 호출을 다른 로봇에게 넘겼으면 바뀐 경로로 간다. 채우기 도크의 commit 구역을 잡았거나 지난 로봇은 방향을 바꾸지 않고 도크 지점까지 간다(`_committed`). 그때 맡은 호출이 없으면 도킹하지 않고 대기 지점으로 간다. 표시 전에 풀리면 아무것도 보내지 않고 멈춤 시계만 0부터 다시 잰다.
- 풀린 뒤 `blocked_rejoin_s` 안에 같은 레인에서 다시 막히면 앞의 시간에 이어 센다. 5초 표시와 60초에 같은 시계를 쓰므로, 표시까지 갔던 레인이 곧 다시 막히면 바로 다시 표시한다.
- `blocked_timeout_s`: ERROR(`BLOCKED_TOO_LONG`, `fault_reason=BLOCKED`). 구역은 쥔다. 레인 표시는 풀림 보고가 오면 ERROR와 상관없이 지운다. `confirm_clear`도 표시를 지운다.
- 표시한 레인은 `blocked_lanes()`와 `robot_view()["blocked_lane"]`로 읽는다. 다른 로봇은 아래 우회 규칙으로 그 레인을 피하고, 길이 없으면 그 구역을 기다린다. `/fleet/robots`의 `fault_reason`은 막힘 동안 `BLOCKED`이고 `fault`는 NONE이다.

우회(11-06 개선 지시서 5번, 레인 막힘 2단계, `traffic/detour.py`): 다음 구역을 남이 쥐고 서 있거나 남은 경로에 막힌 레인이 있으면, 다른 길이 더 쌀 때 돌아간다. 규칙은 결정적이다. 입력과 시계만 쓴다.
- 비용 = 레인 길이 + 들어가는 구역의 벌점. 구역 안의 레인은 길이만 센다. 같은 비용이면 `LaneMap.plan`과 같은 순서로 가른다.
  - 막힌 레인(`blocked_lanes()`)은 못 지난다. 양방향 쌍이면 반대 방향도 막는다. 같은 통로이기 때문이다.
  - 서 있는 로봇이 쥔 구역(그 구역이나 충돌쌍 구역)은 예상 대기 시간 × `detour_speed_mps`. 무한이면 못 지난다. 보유자가 여럿이면 큰 쪽이다.
  - 달리는 로봇이 쥔 구역은 벌점이 없다.
  - 지금 경로에 없던 대기 칸으로는 못 간다. 대기 줄은 앞지르지 않는 줄이라(호출 규칙), 지나가는 로봇이 끼면 호출이 풀릴 수 있다. 특징 지도에서는 이 규칙이 없으면 F1에서 대기 줄을 한 바퀴 도는 우회가 나왔다.
- 서 있음과 달림(`_standing_wait_s`): 열린 이동이나 도킹 goal이 있으면 달린다. 주행 단계(TO_*, DOCKING)에서 goal 사이에 있는 로봇도 같은 tick에 다음 goal을 받으므로 달린다(아래 대기 경우는 빼고). 그 밖은 서 있다. 예상 대기 시간은 위에서부터 먼저 맞는 것이다. 고장(`ERROR`, `OUT_OF_SERVICE`), 두절과 복구 확인, 막힘 기록, `BLOCKED` 보고, 레인 이탈로 섬(13절, 돌아올 때를 모른다)은 무한이다. 단, 막힘 기록 없이 목표를 둔 채 `BLOCKED`인 줄 서기(4절 막힘 처리의 줄 서기, `queue_since`)는 줄 선 시간이다. 줄 선 시간이 `blocked_timeout_s`와 같거나 넘으면, 또는 지금 앞에 정상 로봇이 없으면(낡은 시계) 무한이다. 앞 로봇 뒤에 붙은 로봇을 고장처럼 보면 뒤 로봇들이 괜히 돌아가기 때문이다. 구역을 기다리면 그 구역을 기다린 시간이다. 재시도나 도착 확인을 기다리면 남은 시간이다. 나머지(`AT_STATION`, 대기 칸, 할 일 대기)는 `station_wait_estimate_s`다. pinky_multi_sim 4.2의 "작업 중이면 남은 시간"은 교통이 팔 작업 시간을 모르므로 추정값으로 바꿨다.
- 다시 고르는 때: 현재 구역 말고 쥔 것이 없어 다음 구역을 청할 때뿐이다(`_request_next_zone`). 다음 구역을 이미 쥐었으면 바꾸지 않는다. 다음 구역에 벌점이 없고 남은 경로에 막힌 레인도, 못 비키는(무한) 보유자의 구역(충돌쌍 포함)도 없으면 계산하지 않는다. 계산을 시작할지 정할 때 유한 벌점은 다음 구역만 본다. 멀리 선 로봇은 도착할 때쯤 비킬 수 있기 때문이다. 계산이 시작되면 지금 길과 후보 길이 들어가는 모든 구역에 유한 벌점도 더한다. 무한 보유자를 다음 구역만 보면, 그 앞 구역에 들어선 뒤에는 돌 길이 없을 수 있다(특징 지도 F0에서 F2의 고장 로봇: F1에 들어가면 대기 칸밖에 없다). 기다리는 동안은 tick마다 다시 본다. 대기 시간이 늘어 벌점이 커지면 그때 돌아갈 수 있다.
- 새 길이 지금 길보다 싸야 바꾼다. 같으면 그대로다. 길이 없으면 지금처럼 그 구역을 기다린다. 경로는 반드시 도달할 지점(`_anchor`)부터 바꾸고 목적지는 그대로다. 막힌 로봇 자신과, 표시 때 취소한 goal의 결과를 기다리는 로봇(`_held_for_block`)은 다시 고르지 않는다. 남이 볼 때 결과를 기다리는 로봇은 서 있는 것으로 치고 `station_wait_estimate_s`를 쓴다.
- 왕복 금지: 한 지점에서 포기한 다음 구역은 그 지점을 떠날 때까지 못 지난다. 지시서는 "직전에 포기한 구역"이지만 그 지점에서 포기한 구역을 모두 막는다. 둘 이상을 번갈아 고르지 않게 하려는 것이다. 예외: 지금 길을 못 지나면(비용 무한, 예: 바꾼 길의 보유자가 고장 나거나 막힘) 왕복 금지를 쉬고 다시 고른다. 격리된 로봇의 구역은 `confirm_clear` 전까지 풀리지 않으므로, 막아 두면 우회를 끈 것보다 나쁘게 끝없이 선다. 지금 길의 비용에는 왕복 금지를 넣지 않는다.
- 바꾸면 `DETOUR`(INFO)를 내고 옛 다음 구역 요청을 거둔다. 우회로 경로가 지금 구역 안에서 끝나면 요청 없이 도착한다(전수 검사에서 찾은 경우).
- `detour_enabled=false`면 이 절은 쉬고 동작은 파트 5 전과 같다. 기본 지도와 oneway 지도에서는 돌아갈 길이 없어 켜도 나오지 않았다. 특징 지도의 숫자와 전수 검사의 한계는 11절에 있다.

heartbeat 정지(11-06, 4번): 이동 결과의 reason이 `HEARTBEAT_LOST`면 시도로 세지 않는다. `HEARTBEAT_LOST`(WARN)를 내고 `move_retry_wait_s` 뒤 같은 목표를 다시 보낸다. 그 사이에도 멈춤 시계는 이어 잰다. 끊김이 되풀이되어 `stall_s` 동안 진전이 없으면 `GOAL_STALLED`가 실패 한 번으로 센다.

액션 서버가 없을 때: `_send`에서 `server_is_ready()`가 false인 이동과 도킹 목표는 실패한 시도가 아니다. `on_goal_not_sent`가 목표를 지우고, 거절과 같이 `settle_idx`를 남기고, 같은 시간을 기다리게 하지만 시도 횟수는 늘리지 않는다. `GOAL_NOT_SENT`는 목표가 서버에 닿을 때까지 한 번만 낸다. 보내지 못한 목표는 그 tick 안에 돌아오므로, 다음 tick에 목표가 남아 있거나 그 결과가 오면 닿은 것으로 친다. 팔 목표는 이 길을 타지 않고 지금처럼 `REJECTED`로 시도 횟수에 들어간다.

도착 판정: 이동의 성공 결과가 와도 마지막 보고 위치가 목표 지점에서 `arrival_tolerance_m`보다 멀면 바로 도착으로 치지 않는다. 위치 보고는 결과보다 늦게 올 수 있으므로 `arrival_settle_s` 동안 새 목표를 내지 않고 기다린다. 그 안에 허용치 안의 위치가 보고되면 도착이고, 기한이 지나도 보고된 위치가 멀면 실패한 시도(`NOT_AT_TARGET`)로 친다. 처리는 실패한 이동과 같다. 로봇의 통신 상태가 `COMM_LOST`나 `RECOVERING`이면 위치가 오래된 것이고 복구가 위치를 다시 판단하므로 보지 않는다. 도킹 결과는 위치를 보지 않는다. 정지 오차가 이 값을 자주 넘는 로봇은 재시도를 다 써서 ERROR가 될 수 있으므로, 이 값은 실측한 정지 오차, ε, τ와 함께 정한다. 성공 뒤에 위치 보고가 하나도 오지 않았으면 기한이 지나도 판정하지 않고 기다린다. 그 위치는 성공 전의 것이라 아무것도 말해 주지 않기 때문이다. 보고가 끊긴 로봇은 `COMM_LOST`가 먼저 나고 복구가 위치를 판단한다. 기다리는 동안 새 이동 목표는 내지 않는다(2026-11-06 검토에서 추가). 정지 오차를 넣은 압박 시험은 작업 기록(11-05) 7번에 있고 이 판정을 넣기 전 결과다. 판정을 넣은 뒤의 `basic_orders` 결과는 `docs/worklog_2026-11-06.md`의 압박 시험 표에 있다.

이벤트 코드(traffic, 뜻은 0절): `ROBOT_ISOLATED`, `DOCKED`, `GOAL_RETRY`, `GOAL_FAILED`, `GOAL_STALLED`, `GOAL_NOT_SENT`, `HEARTBEAT_LOST`, `LANE_BLOCKED`, `LANE_CLEARED`, `DETOUR`, `BLOCKED_TOO_LONG`, `COMM_LOST`, `COMM_RESTORED`, `ZONE_WAIT_LONG`, `ZONES_CLEARED`, `CLEAR_REFUSED`, `INPUT_REJECTED`(confirm_clear의 모르는 이름), `ROBOT_REINSTATED`(`FleetManager`가 낸다), `ZONES_STILL_HELD`, `LANE_ERROR`, `BACK_ON_LANE`, `OFF_LANE`, `DEADLOCK_*`, `PLACEMENT_CONFLICT`, `RECOVERY_CONFLICT`, `ROBOT_NOT_REPORTING`.

`TrafficController` 공개 API:
```python
TrafficController(site, params, robots, clock, next_goal_id=None)
on_robot_state(robot, x, y, yaw, state, battery=0.0); on_move_result(robot, goal_id, success, reason)
on_goal_rejected(robot, goal_id); on_goal_not_sent(robot, goal_id)
on_dock_result(robot, goal_id, success, error_x_cm, error_y_cm, error_yaw_deg)
on_dispatch(goal_id, station_id, robot_id=""); on_dispatch_cancel(goal_id); on_release(robot)
on_run_mode(mode); on_robot_fault(robot, reason); confirm_clear(robot, zone_ids, note="")
reinstate_refusal(robot) -> str; reinstate(robot) -> bool
tick() -> list   # one whole cycle, or in parts for FleetManager:
tick_begin(now); tick_progress(now); tick_pick(); step(robot); take_output() -> list
running; robots; everyone_placed; available(robot)
zones_view(); robot_view(robot); status_view(robot)
```
명령: MoveCmd, DockCmd, CancelCmd, DispatchFeedback, DispatchResult, EventOut. 뷰: ZoneView. `status_view`는 `/fleet/robots` 중 traffic만 아는 필드다.

## 5-1. 주문 진행 규칙 (`orders`, FleetCore에서 옮긴 것)
주문 배정:
- **주문은 핑키가 WORK 도크에 도킹을 마친 순간 묶는다.** WORK 요청의 결과가 success로 오면 그 robot_id에 대기 주문 맨 앞을 묶고 `ORDER_BOUND`를 낸다. 그 전에는 "호출"만 한다. 도크 도착 순서가 곧 작업 순서이므로, 이렇게 해야 주문이 등록 순서대로 처리되고 도중에 고장 난 로봇이 주문을 쥐고 있지 않는다.
- 열어 두는 WORK 요청 수 = 아직 묶이지 않은 주문 수. 묶이지 않은 주문이 줄면 남는 요청을 `on_dispatch_cancel`로 취소한다(대기 구간 복귀는 5절의 `commit` 지점 규칙).

주문 상태: PENDING, BOUND, FILLING, TO_OUTBOUND, SWAPPING, 종료(SHIPPED, PARTIAL, HELD). 바구니 상태 `basket[robot]`: EMPTY, FULL, NONE. 모든 핑키는 EMPTY로 시작한다.

FILLING:
1. 남은 품목 = 주문 품목 − 담은 것 − 최종 결품. 남은 것이 있으면 FillCmd를 workcell에 보낸다.
2. 결과 success=true: placed, missing을 누적하고 끝낸다. success=false: placed만 누적하고 재시도(`move_retry_limit` 초과 시 ERROR, 교통이 격리하면 주문은 바로 대기 맨 앞으로 돌아간다). 이 FMS가 STOP으로 취소한 작업의 reason=CANCELED는 시도 횟수에 넣지 않는다. 시간 초과로 취소했거나 서버가 스스로 취소한 것은 실패로 세고 `arm_retry_wait_s`를 기다린다. 교환도 같다.
3. 담은 것이 1개 이상이면 basket=FULL, `on_dispatch(OUTBOUND, robot_id=X)`로 출고 구역으로 보낸다. 0개면 HELD 보고 후 주문을 풀고 basket=EMPTY로 `on_release(X)`. 주문이 `basket_capacity`를 넘으면 도킹 후 팔을 부르지 않고 HELD(detail에 사유). `ship_partial`이 false면 일부만 담겨도 FULL로 보내지 않고 HELD한다(담긴 품목은 detail에 적고 관리자 확인이 필요하므로 ERROR 처리).

SWAPPING(OUTBOUND 요청의 결과를 받은 뒤):
1. SwapCmd(take_full = basket==FULL, put_empty=true)를 arm3에 보낸다.
2. took_full이면 basket=NONE으로 바꾸고 **바로** SHIPPED(결품이 있었으면 PARTIAL) 보고 후 주문을 푼다. put_empty_done이면 basket=EMPTY.
3. 남은 단계만 재시도. 한도 초과면 ERROR(`SWAP_FAILED`). basket=EMPTY가 되면 `on_release(X)`. 그다음 어디로 갈지는 traffic이 정한다(5절의 호출).

팔 실패나 거절 뒤에는 `arm_retry_wait_s`(추정 2초)를 기다렸다가 다시 보낸다. 취소한 작업이 있으면 그 결과가 오거나 `arm_cancel_wait_s`가 지날 때까지 같은 서버에 다음 작업을 보내지 않는다.

팔 시간 초과: `fill_timeout_s`/`swap_timeout_s` 초과 시 취소를 보내고 `arm_cancel_wait_s`(추정 5초) 동안 결과를 기다린다. STOP으로 취소할 때도 같은 시간을 기다린다. 결과가 오면 그대로 처리, 안 오면 ERROR(`ARM_NO_RESULT`, 바구니 내용을 알 수 없으니 관리자 확인). 로봇은 격리되지만 주문은 되돌리지 않는다(아래 자동 격리).

`ARM_NO_RESULT` 뒤에 온 결과: 그 작업을 기억해 두었다가 `ARM_LATE_RESULT`(WARN)로 보고 내용(placed, 또는 took_full과 put_empty_done)을 알린다. 주문이 아직 그 로봇에 묶여 있으면 반영한다. 담기의 placed는 주문에 더하고, FULL 바구니를 꺼낸 교환(took_full)은 제시간에 온 결과처럼 SHIPPED(결품이 있었으면 PARTIAL)로 보고하고 주문을 푼다. 그래서 나중에 로봇을 치워도 주문이 되돌아가지 않는다. 로봇은 ERROR 그대로이고 놓아주지 않으며, 관리자가 확인한다. 이미 로봇을 치웠으면 주문은 되돌아간 뒤이므로 이벤트만 내고 반영하지 않는다. 이때 주문이 두 번 담기거나 출고될 수 있고, detail이 그렇게 알린다. 주문을 이미 보고한 뒤의 교환 재시도라면 묶인 주문이 없으므로 이벤트만 낸다. 로봇을 치우면서 취소한 작업도 같은 방식으로 기억한다. 그 결과가 무엇을 했다고 하면(담은 것이 있거나, `took_full`이나 `put_empty_done`) 반영하지 않고 `ARM_LATE_RESULT`로 알리며, detail은 주문이 두 번 담기거나 출고될 수 있다고 적는다. 아무것도 하지 않았다는 결과는 알리지 않는다(2026-11-06 검토에서 추가).

취소 확인(`fleet_manager` 노드, 이동·도킹·팔 목표 공통): 취소는 그 목표의 결과가 와야 끝난 것으로 친다. 코어는 `CancelCmd`를 내면 잊지만, 노드는 결과가 올 때까지 `cancel_retry_s`마다 취소를 다시 보낸다. 처음 보낸 것을 포함해 `cancel_warn_after`번이 답 없이 지나가면 `CANCEL_UNCONFIRMED`를 한 번 내고, 그 뒤에도 같은 간격으로 계속 보낸다. 다만 취소한 이동·도킹 목표의 결과를 잃으면(결과 future 오류) 결과가 다시 올 수 없으므로 이 경고 다음 재시도에서 그만둔다. 그 로봇의 구역은 코어가 결과나 `confirm_clear`까지 쥔다(5절 레인 막힘). 서버가 수락하기 전에 요청한 취소는 수락되는 순간 보내고, 수락을 기다리는 시간도 횟수에 넣는다. 수락 답이 사라져도 목표는 돌고 있을 수 있기 때문이다. 시각은 노드의 시계를 쓰므로 시뮬레이션 시간을 따른다.

보고 확인(`fleet_manager` 노드): 주문 보고는 `accepted=true` 답을 받을 때까지 노드가 가지고 있고, 답이 없으면 `report_retry_s`마다 다시 보낸다(4절). 서비스가 안 보이는 동안에는 보내지 않는다. 호출이 실패해도 가지고 있다가 다시 보낸다. `accepted=false`면 버리고 `ORDER_REPORT_REFUSED`를 낸다.

STOP: 팔 작업을 취소한다. RUN으로 돌아오면 남은 일부터 다시 한다. PAUSE: 진행 중인 것은 끝내고 새 명령을 내지 않는다.

자동 격리(`on_robot_isolated`, 5절): 교통이 `RobotIsolated`를 보내면 사람을 기다리지 않고 그 로봇의 주문을 대기 맨 앞으로 되돌린다(`ORDER_REQUEUED`, detail에 바구니에 담겨 있던 품목과 격리 이유). 자리는 등록 순서다. 주문은 늘 대기 맨 앞의 것을 묶으므로, 되돌린 주문은 한 번도 묶이지 않은 주문보다 앞서고, 되돌린 주문끼리는 등록 순서를 지킨다(먼저 되돌린 것이 뒤에 등록됐으면 나중에 되돌린 것이 앞선다). 바구니는 NONE으로 둔다. 담겨 있던 품목은 물리적으로 그 로봇에 남고, 같은 품목을 다른 로봇이 다시 담는다. 남은 바구니와 재고는 관리자가 치우며 맞춘다. 진행 중인 담기 작업은 로봇 제거와 같이 취소하고 기억하며 주문은 바로 되돌린다. 늦게 담긴 품목은 격리된 로봇 바구니에 남아 출고되지 않는다(늦은 결과는 `ARM_LATE_RESULT`, detail은 "after the isolation"). 진행 중인 교환 작업은 취소만 보내고 주문은 그 결과까지 쥔다. 팔이 이미 바구니를 가져갔을 수 있기 때문이다. `took_full=true`면 제시간 결과처럼 SHIPPED(결품이 있으면 PARTIAL)로 보고하고 되돌리지 않는다. `took_full=false`면 그때 되돌린다(보통 1초 안). `arm_cancel_wait_s` 안에 결과가 없으면 `ARM_NO_RESULT` 규칙을 따른다(로봇은 이미 격리돼 다시 격리하지 않는다). 두 경우 모두 바구니는 NONE이 되고 로봇을 놓아주지 않는다. 교환 결과를 아직 모르는 주문도 `confirm_clear`는 늘 되돌린다(`ARM_NO_RESULT` 뒤와 같다). 그 뒤 온 `took_full=true`는 `ARM_LATE_RESULT`만 내므로 주문이 두 번 출고될 수 있다. 그대로 두는 이유: 취소 결과는 보통 1초 안에 와서 사람이 치우기 훨씬 전이고, 결과가 끝내 오지 않으면 관리자가 치우면서 바구니를 확인한다. `confirm_clear` 뒤에도 주문을 쥐면 그 주문이 영영 멈출 수 있다(정책, 2026-11-06). 예외: `ARM_NO_RESULT`로 포기한 작업이 지금 묶인 주문에 걸려 있으면 되돌리지 않는다. 팔이 일을 끝냈을 수 있어(교환이면 이미 출고) 되돌리면 두 번 담기거나 출고될 수 있기 때문이다. 이 주문은 늦은 결과로 출고되거나 `confirm_clear` 때 되돌아간다.

로봇 제거(`on_robot_removed`): 묶인 주문이 아직 출고 전이면 대기 주문의 맨 앞(등록 순서 유지)으로 되돌리고 `ORDER_REQUEUED`를 낸다. detail에 담겨 있던 품목을 적는다(되돌리거나 재고를 고치는 것은 관리자 몫). 진행 중이던 팔 작업으로 담긴 품목은 detail에 들어가지 않으므로, 실제 바구니는 관리자가 직접 확인한다. 그 로봇의 팔 작업이 진행 중이면 취소를 보내고 위의 취소 대기 규칙을 따른다. 취소한 작업의 결과가 늦게 와도 주문과 바구니에 반영하지 않는다. 그 결과가 무엇을 했다고 하면 `ARM_LATE_RESULT`로 알린다(위). 같은 제거가 두 번 와도 주문은 한 번만 되돌린다.

이벤트 코드(orchestrator, 심판과 맞추는 약속): `ORDER_BOUND`(detail=주문 ID, source=로봇), `ORDER_REQUEUED`(detail이 주문 ID로 시작), `FILL_RETRY`, `SWAP_RETRY`, `FILL_FAILED`, `SWAP_FAILED`, `ARM_NO_RESULT`, `ARM_LATE_RESULT`, `NO_ROBOT_AVAILABLE`.
노드가 직접 내는 코드: `CANCEL_UNCONFIRMED`, `ORDER_REPORT_REFUSED`(detail이 주문 ID로 시작).
central이 내는 코드: `ORDER_ACCEPTED`, `ORDER_REJECTED`, `ORDER_SHIPPED`, `ORDER_PARTIAL`, `ORDER_HELD`(detail이 주문 ID로 시작), `OUT_OF_STOCK`(재고 기록도 0), `STOCK_MISMATCH`(기록은 있는데 실제로 없음).

`OrderOrchestrator` 공개 API:
```python
OrderOrchestrator(stations, params, robots, clock, next_goal_id=None)
on_order_board(run_mode, orders) # orders: list[OrderRecord(order_id, items: dict[str,int])]
on_dispatch_feedback(goal_id, robot_id, phase); on_dispatch_result(goal_id, success, robot_id, reason="")
on_fill_result(goal_id, success, placed: dict, missing: dict, reason="")
on_swap_result(goal_id, success, took_full, put_empty_done, reason="")
on_robot_removed(robot_id)
check_arm_timeouts(now); step(robot); take_output() -> list
order_of(robot); robot_view(robot)
```
명령: `DispatchCmd(goal_id, station_id, robot_id="")`, `CancelDispatchCmd(goal_id)`, `ReleaseCmd(robot)`, `RobotFaultCmd(robot, reason)`, `FillCmd(goal_id, robot, station_id, order_id, items)`, `SwapCmd(arm, goal_id, robot, station_id, order_id, take_full, put_empty)`, `ReportOrderCmd(order_id, robot, status, placed, missing, detail)`, CancelCmd, EventOut. 카운터 `OrderOrchestrator.shipped`.

FMS(`FleetManager`, 노드 `fleet_manager`): 두 모듈을 묶어 명령을 서로에게 전달하고, 바깥 ROS 인터페이스를 맡는다. 중계 노드가 없으므로 합치기 전 판의 confirm_clear 콜백 그룹 문제는 사라진다. `FleetManager`는 한 호출 동안 시각을 한 번만 읽고, `record_to(write)`로 모든 입력과 그 결과를 JSON 한 줄씩 남긴다(`fleet_manager_node --record PATH`, `pc.launch.py record:=PATH`). 형식은 `tools/replay.py` 머리말에 있다. 노드가 직접 내는 `CANCEL_UNCONFIRMED`, `ORDER_REPORT_REFUSED`와 그 판단의 근거인 central의 보고 답, 취소 요청의 답은 코어를 거치지 않으므로 기록에 없고 재생에서 비교하지 않는다. 두 이벤트는 `/central/events`로 나가므로 bag에는 남는다. `robots_view()`가 `/fleet/robots`의 내용을 만든다.

## 6. 시나리오 형식 (`sim/fake_robots/scenarios/*.yaml`)
```yaml
seed: 7
speed_factor: 5.0
timeout_s: 900
stock: {arm1: {A: 6, B: 6}, arm2: {C: 6, D: 6}} # 실제 물건. 가짜 작업공간은 팔별로 나눠 둔다(실제 배치는 공유 영역 1곳, 관제 로직과는 무관)
inventory: {A: 6, B: 6, C: 6, D: 6} # 중앙의 재고 기록. 생략하면 stock 합계
orders:
  - {at_s: 0, id: O-001, items: {A: 1, C: 1}}
robots:
  pinky1: {start: P-6, speed_mps: 0.20, pose_noise_m: 0.03} # report_delay_s(보고 지연, 시나리오 초), stop_error_m(정지 오차, 축마다 표준편차)도 쓸 수 있다. 기본 0
arm: {seconds_per_item: 4.0, swap_seconds: 8.0, fail_rate: 0.0}
dock: {duration_s: 3.0, error_cm: 1.0}
faults: [] # comm_loss, nav_abort(target: 지점이면 그 지점 목표만), stuck 유지. 추가: arm_fail(대상 arm, count), swap_partial(count), operator_remove(로봇을 코스에서 치우고 confirm_clear 호출), drift(robot, duration_s, offset_m: 그동안 보고 위치만 진행 방향 왼쪽으로 민다), operator_reinstate(start 지점에 다시 놓고, 앞선 confirm_clear가 받아들여진 뒤 reinstate 호출, basket 기본 EMPTY. 그 로봇을 치운 뒤에만. 그 전의 stuck도 끝낸다). expect `bound_after_reinstate_min`: 다시 투입한 로봇이 그 뒤 묶은 주문 수의 하한, `shipped_after_reinstate_min`: 그중 ORDER_REQUEUED로 돌아가지 않고 출고(SHIPPED·PARTIAL)된 주문 수의 하한
expect: {all_orders_finished: true, orders_shipped: 6, orders_partial: 0, orders_held: 0, fifo_violations: 0, dual_arm_overlaps: 0,
  zone_holder_max: 1, released_without_position_check: 0, unheld_zone_pose: 0, released_while_off_lane: 0, path_not_connected: 0, events_include: []}
```
`expect`의 숫자는 정확히 일치해야 통과한다(`judge.py`의 `summary`). `released_while_off_lane`도 정확히 일치(기본 0)이고, drift 일정으로 판정한다(아래 심판 추가 항목). `path_not_connected`도 정확히 일치(기본 0)다. 예외: 이름이 `_min`으로 끝나는 `orders_shipped_min`, `direct_returns_min`, `shipped_before_operator_min`, `detours_around_isolated_min`은 최솟값이다(기본 0). `orders_shipped`, `orders_partial`, `orders_held`는 적지 않으면 보지 않는다. `events_include`는 나왔는지만, `event_counts`는 코드마다 정확한 수를 본다. 고장이 있는 시나리오는 `orders_shipped`나 `orders_shipped_min`이 1 이상이어야 읽힌다("0건이어도 통과"를 막는다).

시나리오 파일의 `map`(선택)은 `sim/fake_robots/maps`의 지도 이름이다. 있으면 `pc.launch.py`와 `fakes.launch.py`가 `map_file`을 주지 않았을 때 그 지도를 모든 노드에 준다. 없으면 기본 지도다.

특징 지도 시나리오 3개(11-06, 5번과 5.5번): `feature_basic_orders`(`basic_orders`와 같고 지도와 출발 칸만 다름), `feature_blocked_lane`(`blocked_lane`과 같고 지도, 출발 칸, 물체 시각(46초부터 30초)이 다름, `DETOUR`를 기대함), `feature_auto_isolation`(레인 F1->F2 위에서 격리된 로봇을 좁은 통로로 돌아가 관리자 전에 전부 출고). 통합 시험 목록(`test_scenario_integration.py`)에는 넣지 않았다. 전체 시험 시간을 늘리지 않으려는 것이다. 따로 돌린 결과는 작업 기록(파트 5, 5.5)에 있다.

기본 지도 시나리오 10개: `basic_orders`(주문 6건, 고장 없음), `order_burst`(한꺼번에 8건, 진행 중 추가 주문, 밀린 주문이면 대기 구간을 거치지 않고 바로 작업공간으로 가는지 확인), `shared_lane_failure`(통신 두절+주행 실패), `stock_shortage`(재고 0인 품목과 기록만 있고 실물 없는 품목: PARTIAL과 HELD, OUT_OF_STOCK과 STOCK_MISMATCH 구분), `stuck_robot`(차선에서 멈춤: 뒤따르는 로봇이 막히고 경고가 나는지), `operator_recovery`(팔 고장 3회 → ERROR와 자동 격리 → 관리자가 치움 → 주문이 다시 처리되어 전부 끝남), `blocked_lane`(`basic_orders`에 pinky1 앞 물체: 5초 표시와 취소, 풀린 뒤 같은 목표로 이어 가 전부 출고. pinky3은 처음 2초 `LOCALIZING`), `auto_isolation`(팔 고장 3회 → 자동 격리 → 앞서 간 로봇은 사람 없이 출고 → 관리자가 치운 뒤 되돌린 주문부터 전부 끝남, 11-06 5.5번), `off_lane`(한 대의 보고가 레인 옆으로 밀림 → 세우고 구역 유지 → 뒤 로봇은 그 구역 앞에서 대기 → 돌아오면 재개 → 전부 출고, 11-06 파트 3), `reinstate`(관리자가 치운 로봇을 빈 바구니와 다시 투입, 다시 오른 로봇도 주문을 받아 7건 모두 출고, 11-06 6번).

심판은 FMS가 내는 이벤트를 본다. 가짜 노드(FakePinky, FakeWorkcell, FakeArm3, FakeVision, FakeCentral)는 인터페이스가 같으므로 그대로 쓴다. 시나리오 시각은 시뮬 시계(`speed_factor` 5배속)이고, heartbeat만 벽시계다(4절). `tools/pressure_sweep.py`는 시나리오에 지연과 잡음을 넣어 돌리고 표를 만든다(7번 표는 작업 기록에 있다).

심판 추가 항목: `shipped_before_operator`(첫 `ROBOT_ISOLATED`부터 격리된 로봇(이벤트 source)의 첫 `ZONES_CLEARED` 전까지 출고·일부 출고된 건수. 기대값 `shipped_before_operator_min`, 기본 0), `detours_around_isolated`(격리된 로봇이 치워지기 전, detail의 `held by <로봇>` 중 하나라도 그 로봇인 `DETOUR` 수. 이벤트 하나는 한 번. 기대값 `detours_around_isolated_min`, 기본 0), `fifo_violations`(ORDER_BOUND 순서가 등록 순서와 다르거나 SHIPPED/PARTIAL 순서가 묶인 순서와 다르면 위반. ORDER_REQUEUED된 주문은 다음에 묶이는 주문이어야 하고, 되돌림 없이 두 번 묶이면 위반), `dual_arm_overlaps`(arm1과 arm2가 동시에 PLACING. 양쪽 팔이 필요한 주문인데 PLACING이 안 보이면 실패. 2026-11-04에 양팔 동시 계획으로 정했으므로 이 항목의 의미가 바뀐다: 동시 PLACING은 허용, 충돌만 위반. 가짜 workcell과 심판을 그에 맞춰 고치기 전까지 기대값은 참고용), `unexpected_errors`(시나리오가 예상하지 않은 ERROR 이벤트), `order_results`(주문별 결과), `released_while_off_lane`(drift 시작 1초 뒤부터 끝날 때까지 그 로봇이 놓은 구역 수. 1초는 FMS가 연속 보고로 알아챌 여유), `path_not_connected`(detail에 `PATH_NOT_CONNECTED`가 든 `GOAL_RETRY`·`GOAL_FAILED` 수, 곧 관제가 처리한 이 이유의 실패 수. 이미 바뀐 목표의 결과는 관제가 버려 세지 않는다. `null`이면 보지 않는다. 11-08, 지시서 7.5), 시간 초과는 시나리오가 예상한 경우 외에는 실패.

## 7. 알려진 약점 (이번 작업에서 고치지 않고 문서에 남김)
- `arrival_tolerance_m` 안에 멈춰 도착으로 쳤는데 레인 진행이 `s ≤ d`이면 뒤 구역이 해제되지 않아 조용히 멈출 수 있다. 가장 짧은 레인에서 d를 뺀 값이 허용치보다 작은 지도에서만 생긴다. 기본 지도는 0.4 m에서 0.2 m를 빼 0.2 m이므로 0.15 m에 해당하지 않고, 시험으로 고정했다. 시험 격자(0.33 m)와 `sim/fake_robots/maps/oneway_*.yaml`은 조건을 만족하지 않는다. 시작할 때 막지 않는다. 바닥 선 기준까지 넣은 숫자와 안은 13절 "도착 허용과 해제"(사용자 결정 대기).
- 이동이나 도킹 서버를 찾은 뒤 사라지면 로봇은 구역을 쥔 채 `move_retry_wait_s`마다 다시 보낸다. 시간 제한은 없고 `GOAL_NOT_SENT`를 한 번 낼 뿐이다. 서버가 돌아올지 관리자가 치울지는 사람이 본다.
- 교착 포기 대상 로봇의 위치 보고가 끊기면 해제에 필요한 보고가 모자라 `_finish_retreat`가 재확인을 반복한다. `comm_timeout_s`가 지나 `COMM_LOST`가 되면 그 로봇의 대기 요청이 모두 취소되어 원래 대기 고리가 끊어지고, 고리가 하나도 남지 않으면 `_check_deadlock`이 재확인 상태를 지운다. 다른 고리가 남아 있으면 재확인이 이어질 수 있고, 그동안 새 호출과 새 이동 단계도 나가지 않는다. 옛 입출고 코드에는 이 재확인을 끝내는 수정이 있지만 GG 코드에는 들어오지 않았다.
- FMS가 멈추면 주문 진행, 구역 잠금, 교착 처리가 함께 멈춘다. 화면에 로봇 상태가 오지 않고 `confirm_clear`와 STOP/PAUSE도 전달되지 않는다. 중앙이 직접 내는 이벤트는 계속 나온다. 한 프로세스로 합친 대가다. 핑키는 `/fleet/heartbeat`가 끊기면 스스로 선다(제안, 4절). 팔의 목표는 이 대상이 아니라 담기 중에 FMS가 죽으면 팔은 하던 일을 계속한다. 비상 정지는 FMS에 기대지 않도록 따로 정해야 한다(SR-36, 13절).
- `use_fake:=false`에서 `/central/orders` 발행자가 없으면 경고 없이 아무것도 움직이지 않는다.
- ~~`confirm_clear`에 없는 구역을 주면 KeyError로 노드가 죽을 수 있다.~~ 닫힘(11-06, 1번): 모르는 로봇이나 구역이 하나라도 있으면 아무것도 바꾸지 않고 `accepted=false`와 `reason`으로 답하며 `INPUT_REJECTED`를 낸다(`recovery.py`의 `confirm_clear`, 5절).
- ~~이동/도킹 목표에 관제 쪽 시간 제한이 없어 결과가 안 오면 조용히 멈춘다(FSM 책임).~~ 닫힘(11-06, 2번): 목표가 열린 채 제자리면 멈춤 감시가 취소하고 다시 보내며, 세 번이면 ERROR다(5절). 남은 것: 계속 움직이지만 끝내지 않는 로봇(한 점에서 `stall_move_m` 넘게 오가는 경우)과 0.01 m/s 넘게 기어가며 끝내지 않는 로봇은 잡지 못한다. 위치 잡음이 σ 0.05 m 이상이면 잡는 데 더 오래 걸릴 수 있다.
- ~~레인을 크게 벗어난 로봇도 s > d면 구역을 놓는다(`LANE_ERROR`는 알리기만 한다).~~ 닫힘(11-06, 3번): 이탈을 잰 보고에서 e가 `lane_error_warn_m`을 넘는 것이 같은 레인에서 `release_confirm_samples`번(기본 3) 이어지면 목표를 취소해 세우고 구역을 쥐며, 이탈 중에는 해제를 멈춘다(5절). 남은 것: 그 횟수를 채우기 전의 보고로는 아직 s > d 해제가 될 수 있다. 막힘 표시 뒤처럼 목표가 없는 동안은 이탈을 재지 않는다. 통신 두절이 겹치면 복귀 뒤 같은 목표 대신 통신 복구 규칙의 경로로 간다.
- 위치 잡음으로 조기 해제가 가능하다(필터 없음). 시험 무위반이 정지 상태 안전을 증명하지는 않는다. 어느 잡음까지 0인지는 작업 기록 7번의 표에 있다.
- `robot_radius_m` 0.08은 트레이를 뺀 값이다. 트레이를 포함한 길이를 재서 d와 충돌쌍을 다시 계산해야 한다. r이 커지면 충돌쌍이 늘어난다.
- FMS를 재시작하면 진행 중이던 주문과 할당, 바구니를 기억하지 못한다. 전원 보고부터 다시 시작하고, 같은 바구니를 다시 채울 수 있다. 실물에서 어떻게 할지는 정하지 않았다.
- `RECOVERY_CONFLICT`(복구한 로봇이 남의 구역 안에 있음)는 그 로봇을 격리하고 다른 로봇을 세우지 않는다. 시작 때의 `PLACEMENT_CONFLICT`와 다르다.
- ~~고장 난 로봇의 주문은 관리자가 치울 때까지 멈춘다.~~ 닫힘(11-06, 5.5번): 로봇 1대가 고장 나도 주문은 사람을 기다리지 않는다(자동 격리, 5절). 다만 기본 지도처럼 우회로가 없는 곳에서는 격리된 로봇 뒤의 로봇이 관리자가 치울 때까지 기다린다. 격리된 로봇이 쥔 구역은 우회 계산에서 무한 벌점이라, 그 구역을 지나지 않는 길이 있을 때만 돌아간다. 특징 지도에서 확인했다(`feature_auto_isolation`): F1, F2를 쥐고 서면 좁은 통로(F0 T2 T1 QW W)로 돌아가고, 도크(W, O)나 하나뿐인 길(동쪽행 E, O->F0, 대기 줄 출구 QW)을 쥐면 기다린다. F2만 쥔 경우도 `df2a02b`부터 남은 경로의 무한 보유자를 보므로 O에서 돌아간다(SimWorld, 작업 기록 파트 5.5). 남은 일: 대기 줄로 들어가는 결정은 출구(QW)가 막힌 것을 보지 않는다. 격리된 로봇이 QW만 쥐면 대기 줄에 들어간 로봇이 갇힌다(SimWorld 6건 중 5건).
- `ARM_NO_RESULT`로 격리된 로봇의 주문은 되돌리지 않고 관리자를 기다린다. 격리 때 교환 중이었으면 그 결과까지 기다리고, 결과가 없으면 같은 규칙이다(5-1절의 예외). 두 번 출고를 막으려는 것이다.
- 격리로 되돌린 주문은 다른 로봇이 처음부터 다시 담는다. 격리된 로봇의 바구니에 남은 품목은 관리자가 치우며 재고를 맞춘다. FMS는 바구니를 NONE으로만 기록한다.
- 보고와 취소를 다시 보낼 때마다 rclpy 클라이언트에 응답 대기 항목이 하나씩 남고, 답이 오지 않은 요청은 프로세스가 끝날 때까지 남는다. central이나 서버가 발견은 되는데 답하지 않는 동안 보고는 `report_retry_s`, 취소는 `cancel_retry_s`마다 1개씩 늘어난다. 먼저 보낸 요청을 지우면 응답이 재전송 간격보다 느린 central의 답을 영영 받지 못하므로 그대로 두었다.
- `CANCEL_UNCONFIRMED`는 알리기만 한다. 취소가 끝내 듣지 않는 로봇을 FMS가 세울 방법은 없고, `move_to`와 `dock`에는 다른 정지 수단이 없다.
- central이 거절한 보고(`ORDER_REPORT_REFUSED`)의 주문은 보드에 남아 있어도 아무도 맡지 않고 보고하지도 않는다. 관리자가 처리한다.
- `FakePinky`는 즉시 선다. 실물의 감속 거리는 `heartbeat_timeout_s` 동안 이동 거리에 더해야 한다.

## 8. 물리 위험 (실물 시험 전 미확인)
- 팔이 바구니+상품을 함께 들어야 한다. 그리퍼 20~45 mm, 물체 무게 약 150 g(02 문서에는 약 50 g), 실측 없음. 바구니 손잡이 형상과 총 무게 시험이 먼저다.
- 사용 도달 거리 약 20~25 cm라 팔 베이스를 도크에 바짝 붙여야 하고, 도크 정지 오차가 작아야 한다.
- 양팔 동시 궤적 계획(MoveIt 플래닝 그룹)과 `follow_joint_trajectory` 실행은 시험한 적이 없다. arm_control이 pymycobot으로 받은 궤적을 얼마나 따라가는지 실측이 없다.

## 9. 작업 규칙
- 실물 로봇 접속과 bringup 금지. Confluence 수정 금지. git은 개인 저장소 `PJW-1/roscamp-repo-2`의 브랜치 `fms-work-order`에만 올린다(2026-11-05 사용자 지시). `main`과 팀 저장소에는 올리지 않는다.
- ROS를 띄우는 모든 명령은 `ROS_DOMAIN_ID`를 아래 지정 값으로, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 격리한다(기본값 6은 로봇 네트워크).
- 코드 주석과 docstring은 짧은 한국어, 식별자는 영어(2026-11-06 사용자 지시, 지시서의 "주석 영어"보다 우선). README와 문서는 한국어.
- 프로토타입을 실행한 PC: Jazzy, colcon, pytest 7.4.4. rmw_cyclonedds 없음. 실행과 CI 기준은 Fast DDS(`rmw_fastrtps_cpp`)다. 옛 결과는 WSL2+CycloneDDS였으므로 환경이 다르다고 밝힌다. 실물에 쓸 DDS는 T-10 시험 뒤에 정한다.
- 결과를 부풀리지 않는다. 실행 안 한 것은 안 했다고 쓴다.

## 10. HW와 실행 위치 (`시스템_아키텍처.html` v18 기준)
| 장치 | 대수 | 구성 | 실행하는 소프트웨어 | 네트워크 |
|---|---|---|---|---|
| 관제 PC | 1대 이상 | 사양 미정 | 화면, central, FMS(`fleet_manager`) | 유선 |
| 공유기 | 전용 1대 | 기종과 위치 미정 | 없음 | |
| Pinky Pro (pinky1~3) | 3대 | Raspberry Pi 4B(메모리 4 GB), 라이다 RPLidar C1, 구동 모터 Dynamixel 2개, 카메라 OV5647, 관성 센서 BNO055, 배터리 센서 ADC | pinky_bringup, pinky_fsm, pinky_dock, Nav2 + AMCL | Wi-Fi |
| JetCobot 작업공간 (arm1, arm2) | 2대 | MyCobot 280(6축 + 그리퍼), 카메라 640×480, 팔마다 팔 PC 1대(사양 미정) | 팔마다 arm_control, arm_vision. workcell은 두 팔 PC 중 1곳(13절). v18은 이것을 `arm_planner (workcell)`(임시 이름)로 그리고, 두 팔의 배분과 궤적 계획, Vision 결과 받기를 맡긴다. Vision 서버는 제안 | 팔 PC 유선, 로봇 Wi-Fi |
| JetCobot 출고 구역 (arm3) | 1대 | 같은 구성 | arm_control(`swap_basket` 제공) | 팔 PC 유선, 로봇 Wi-Fi |

- 사양을 실물로 확인한 것은 핑키 1대(`pinky_06c5`)와 제트코봇 1대(`jetcobot_126b`)뿐이다. 나머지 4대는 같다고 가정했다.
- 전용 공유기 1대로 묶기로 했다(2026-11-04). 지금은 로봇마다 자기 핫스팟을 쓰고 있다.
- 실행 위치는 2026-11-04에 정했다: Nav2는 핑키 Pi, 상품 인식과 workcell은 팔 PC.
- 네트워크: v18은 "PC 6대는 전부 유선, 로봇 6대(핑키 3, 제트코봇 3)는 Wi-Fi"(2026-11-05 확인)라고 적는다. 제트코봇의 Raspberry Pi 5는 그림에 없고, 고정 장비라 유선으로 잇기를 권한다. 이전 판의 "유선"은 팔 PC만 가리켰다.
- 관제 PC 행은 이 문서의 배정이다. v18은 서버 구역에 central, DB, FMS, Vision 서버를 그리고, PC 6대의 역할 배정이 정해지면 다시 그린다고 적었다. FMS 안은 core, pinky, arm 세 모듈이고 arm은 `fill_basket`·`swap_basket` 클라이언트다(0절의 대응). workcell은 FMS에 합치지 않는다.

## 11. 코드의 현재 상태 (저장소 `PJW-1/roscamp-repo-2`, 브랜치 `fms-work-order`, 2026-11-06)
**FMS는 코드에서도 두 모듈로 나뉘었다.** 11-05에 `FleetCore`와 `fleet_manager_node`를 `orchestrator_core`, `traffic_core`, `FmsCore`, `fms_node`로 나눴고, 11-06 개선 지시서 0.5번에서 이름과 폴더를 역할에 맞췄다. 지금은 `pc/fleet_core/fleet_core/` 안에 `manager.py`(`FleetManager`), `orders/orchestrator.py`(`OrderOrchestrator`), `orders/validate.py`, `traffic/`(`controller.py`의 `TrafficController`와 `mission.py`, `recovery.py`, `zones.py`, `selector.py`, `deadlock.py`, `lane_map.py`, 11-06에 더한 믹스인 `blocking.py`(파트 4), `detour.py`(파트 5), `isolation.py`(파트 5.5), `off_lane.py`(파트 3)), `common/`(`commands.py`, `params.py`, `contract.py`, `contract_check.py`, 입력 검사 `validate.py`(파트 1)), `data/`가 있고, 노드는 `pc/fleet_manager/fleet_manager/fleet_manager_node.py`(노드 이름 `fleet_manager`)다. 바꾼 이름의 전체 표는 `docs/worklog_2026-11-06.md`에 있다. 지도 파일의 키 이름(`nodes:` → `waypoints:`, 옛 키는 로더가 거절한다)과 heartbeat의 이름 말고 0.5에서 동작은 바꾸지 않았다(그때 전체 시험 875건 통과, 로봇 2대 전수 검사 다섯 경우의 숫자가 0.5 전과 같음). 그 뒤 파트 1, 2, 4, 5, 5.5, 3이 동작을 더했다(아래 표). 옛 호출 방식을 쓰는 시험을 그대로 돌리려고 시험 폴더에 얇은 `legacy_core.py`만 남겼다. 경계는 `test_fms_boundary.py`가 import 그래프로 검사한다: `traffic` 패키지의 어느 모듈도 `orders`나 `manager`에 닿지 않는다.

| 항목 | 코드 |
|---|---|
| 주문 입력 | `fleet_manager` 노드가 `/central/orders`를 구독해 `FleetManager.on_order_board`로 넘긴다. run_mode는 `traffic`에도 간다 |
| 핑키 호출 | `orders`가 묶이지 않은 주문 수에 맞춰 `DispatchCmd`, `CancelDispatchCmd`를 낸다. `traffic`이 `tick_pick`에서 `RobotSelector.pick`으로 로봇을 고른다 |
| 주문 묶기 | `orders`가 작업공간 `DispatchResult`의 success를 받은 뒤 맨 앞 주문을 묶는다 |
| 담기, 교환 | `orders`가 `FillCmd`, `SwapCmd`를 내고 노드가 `/workcell/fill_basket`, `/arm3/swap_basket`을 호출한다. `traffic`은 팔을 모른다 |
| 팔 재시도, 시간 초과, 바구니 상태 | `orders` |
| 최종 보고 | `orders`의 `ReportOrderCmd`를 노드가 `/central/report_order`로 보낸다 |
| 고장 로봇의 주문 | 지도에 오른 로봇이 ERROR가 되면 `traffic/isolation.py`가 격리하고 `RobotIsolated`로 `orders`에 알려 주문을 바로 되돌린다(`ORDER_REQUEUED`, 파트 5.5). 예외: 교환 중이면 그 결과까지 쥔다. `ARM_NO_RESULT`가 걸린 주문은 `confirm_clear`까지 쥐고, 그 전에 찬 바구니를 꺼냈다는 늦은 교환 결과(`took_full`)가 오면 출고로 보고한다(5-1절). 격리 없이 치운 로봇은 `FleetManager.confirm_clear`가 `on_robot_removed`를 부른다 |
| 잘못된 입력 | `FleetManager`와 `common/validate.py`, `orders/validate.py`가 거부하고 `INPUT_REJECTED`를 낸다(파트 1) |
| 멈춤 감시, 레인 이탈, 막힘, 우회 | `controller.py` `_watch_stall`(파트 2), `traffic/off_lane.py`(파트 3), `traffic/blocking.py`(파트 4), `traffic/detour.py`(파트 5). 규칙은 5절 |
| 이동 목표(레인 준수) | `traffic/lane_path.py`(파트 7). `lane_goal_mode: path`면 잠금을 쥔 데까지 레인 폴리라인을 `MoveCmd.path`로 내고 노드가 `MoveTo.path`에 싣는다. 해제 보강은 `traffic/release_floor.py`. FakePinky는 선의 지점을 차례로 따른다. 규칙은 5절 |
| 치운 로봇 재투입 | `FleetManager.reinstate`(`/fleet/reinstate`, 관제 요구)가 traffic `reinstate.py`와 `on_robot_reinstated`를 부르고 `ROBOT_REINSTATED`를 낸다. 시나리오 `reinstate` |
| 교통 모델 | 레인 지도(3절). 잠금은 구역, 해제는 레인 투영(5절), 충돌쌍과 양방향 묶음은 지도에서 계산한다 |
| 로봇별 상태 | `/fleet/robots` 1 Hz. `/fleet/assignments`, `/fleet/zones`는 그대로 |
| 자체 정지 | `/fleet/heartbeat` 2 Hz(11-06에 `/fms/heartbeat`에서 이름만 바꿈, 관제 요구). `FakePinky`에 구현했고 실물은 FSM 담당 몫이다 |
| 기록과 재생 | `fleet_manager_node --record`, `tools/replay.py` |
| 중앙과 FMS 사이 | `fleet_manager` 노드가 `/fleet/*`, `/central/events`를 발행하고 `/fleet/confirm_clear`를 제공한다. 가짜 중앙은 `/fleet/*`를 구독하지 않고, 심판 `scenario_monitor`가 구독하며 관리자 역할로 confirm_clear를 부른다 |
| 중앙과 팔 사이 | `contract.yaml`에 `/{arm}/state`, `/{arm}/vision/items`가 있다. FMS는 이 토픽을 판단에 쓰지 않는다. `scenario_monitor`가 심판으로 `/{arm}/state`를 구독한다 |
| 인터페이스 | `contract.yaml` 0.4-draft. 0.4(11-07)는 `/fleet/heartbeat`를 HEARTBEAT 프로필(reliable, depth 1)로 바꾸고 `MoveTo` 결과 reason `PATH_NOT_CONNECTED`를 더했다(지시서 1.5, 7.5, 제안). 11-05에 더한 것은 `/fleet/robots`, `/fms/heartbeat`(11-06에 `/fleet/heartbeat`로 이름 변경) 둘이다. 11-06에 상수 `RobotState.STATE_BLOCKED`, `STATE_LOCALIZING`, `MoveTo` `REASON_HEARTBEAT_LOST`(파트 4), `ConfirmClear` 응답의 `reason`(파트 1), `MoveTo` goal의 `geometry_msgs/Pose[] path`(파트 7), 서비스 `/fleet/reinstate`(`Reinstate.srv`, 파트 6), `real_only` 액션 `follow_path`를 더했다. 0.3에 더한 것은 모두 관제 요구(팀 합의 전)다. 그 밖의 이름, 필드, QoS는 그대로다 |
| 양팔 규칙 | `README.md`, `fake_workcell_node`, 심판 `dual_arm_overlaps`는 아직 동시 PLACING을 위반으로 본다. 지시서가 심판을 고치지 말라고 했으므로 그대로 뒀고 2026-11-04 결정(6절)과 다르다 |
| 중앙, 핑키, 팔 | `pc/central`, `pinky/*`, `arm/*`는 빈 패키지이고 가짜 노드만 동작한다 |
| GitHub | 개인 저장소 브랜치에만 올렸다. 팀 저장소 `addinedu-roscamp-16th/roscamp-repo-2`에는 올리지 않았다 |

검증은 세 층이다(11-05 PDF). 같은 규칙을 전수 검사(`tools/model_check.py`, 설계), 가짜 노드 시나리오(`sim/fake_robots/scenarios`, 통합), 기록 재생(`tools/replay.py`, 현장)으로 확인한다. 실물 기록은 아직 없어서 세 번째 층은 가짜 노드의 기록으로만 돌렸다. 이 프로토타입의 숫자는 모두 가짜 로봇에서 나온 것이다.

결과(전부 가짜 로봇. 숫자를 얻은 코드는 줄마다 적었다):

| 검증 | 결과 |
|---|---|
| 최종 전체 시험(파트 8) | `a12b159`(git archive 사본, 커밋과 diff 0, 주 세션, `ws_final`, `full_final.log`. `e30c31d`와는 문서 3개만 다르다): 빌드 11개 통과, 1401건, 실패 0, 오류 0, 건너뜀 0(`fleet_core` 944, `fake_robots` 366, `fleet_manager` 91), 시험 1023 s. 그 전: `4766665` 1380건 0 실패(1019 s). `4b2b1ba` 1384건 중 1 실패(`test_scenario_integration[reinstate]`, 시나리오 모니터의 치우기와 배정의 경쟁, `893329e`의 첫 치우기 대기로 고침). `893329e` 시나리오 10회 모두 통과, 다시 치우기 0, `ZONES_STILL_HELD` 0, `COMM_LOST` 0 |
| 단위·통합 시험 | `0cb8525`(파트 6과 7 후속까지. 그 뒤 `f8c2eea`는 시험 설명 1줄, `6ab04fe`는 worklog뿐이다. 주 세션 확인, `p6q_full.log`): 빌드 통과, 1376건, 실패 0, 오류 0(`fleet_core` 944, `fake_robots` 341, `fleet_manager` 91), 시험 1017 s. 이전: 파트 7 `cd05720` 1294건, 파트 3 `ecab72c` 1237건, 0.5 뒤 875건, 파트 1·2 뒤 `533d351` 1036건, 파트 4 `a5a4745` 1100건, 파트 5 `df2a02b` 1163건, 파트 5.5 `33f01d7` 1199건(작업 기록) |
| 시나리오 10개(기본 지도, 실제 launch, 통합 시험 목록, 도메인 50 + 순번) | `0cb8525` 전체 시험 안에서 10개(`basic_orders`, `order_burst`, `shared_lane_failure`, `stock_shortage`, `stuck_robot`, `operator_recovery`, `blocked_lane`, `auto_isolation` 57, `off_lane` 58, `reinstate` 59) 모두 통과. 출고 숫자는 따로 돌린 실행에서 적었다. `df2a02b`(`run_scenarios.sh`): `basic_orders` 6/6, `order_burst` 10/10, `shared_lane_failure` 6/6, `stock_shortage` 2/7, `stuck_robot` 2/6(시간 초과, 기대값), `operator_recovery` 4/4, `blocked_lane` 6/6(`LANE_BLOCKED` 1, `LANE_CLEARED` 1), 모두 심판 통과, 최소 해제 거리 0.165~0.187 m. `auto_isolation`은 파트 5.5 트리(`5ebd821` 반영분)에서 4/4, `shipped_before_operator` 1 |
| 시나리오 `off_lane`(파트 3, 통합 시험 목록 9번째, 도메인 58) | `c99de32`(파트 3 단독): 통과, 6/6, 7.36초 `LANE_ERROR`(e 0.29 m), 15.35초 `BACK_ON_LANE`, 그동안 구역 S·L을 쥐고 pinky2는 S를 기다림, `released_while_off_lane` 0, 조기 해제 0. 파트 4·5 병합 트리에서도 통과(6/6). 그때 전체 시험은 `c99de32` 1064건 실패 0. 파트 3을 합친 `ecab72c`의 전체 시험(위 줄)에서도 통과 |
| 시나리오 `reinstate`와 path 모드 셋(`0cb8525`와 같은 코드, 주 세션, `p6q_scen.log`) | `reinstate` point 7/7, path 7/7(둘 다 `bound_after_reinstate_min` 1 충족). path 모드 `feature_auto_isolation` 6/6(`shipped_before_operator` 5), `feature_blocked_lane` 6/6, `off_lane` 6/6. 모두 심판 통과, 위반 [] |
| 시나리오 12개 × 두 모드(파트 7, `faea3a5`) | 기본 지도 9개와 특징 지도 3개를 point 12판, path 12판으로 돌렸다. 24판 모두 심판 통과, 조기 해제 0, 근접 0, 두 모드의 처리 시간 차는 3 s 안. 24개 기록은 `cd05720`에서도 같게 재생된다. 표는 작업 기록 파트 7 절 |
| 시나리오 3개(특징 지도, 통합 시험 목록 밖) | `df2a02b`: `feature_basic_orders` 6/6, `feature_blocked_lane` 6/6(`DETOUR` 1, `LANE_BLOCKED` 1, `LANE_CLEARED` 1). `feature_auto_isolation`은 파트 5 최종과 합친 5.5 트리(`3acc65a` 뒤, 정확한 커밋은 작업 기록에 없다)에서 통과: 102.7초 격리, `DETOUR` 3(`detours_around_isolated` 3), `shipped_before_operator` 4, 6건 모두 194.6초까지 출고, 400.2초 `ZONES_CLEARED` |
| 전수 검사, 로봇 2대 (상태, 교착, 기아) | `fa94eb6`. 11-06에 `86b1fe1`, 0.5 정리 뒤의 트리, `533d351`, `eb50ef2`, `bc784a9`, `df2a02b`(우회 켬, 파일 값)에서 다시 돌려 같은 값을 얻었다. `oneway_loop` (34407, 0, 0). `oneway_shortcut` 묶음 끔 (69497, 36, 30), 켬 (62615, 0, 0). 기본 지도 끔 (36573, 432, 396), 켬 (37078, 0, 0). 충돌쌍을 넘는 FIFO에서도 켬은 0이다 |
| 전수 검사, 로봇 3대 | `oneway_loop`은 `c9f46ab` 전의 복사본, 나머지는 `c9f46ab`이고, 기본 지도 묶음 켬만 `fc133b1`(코드는 `b6d75d9`와 같다)에서 다시 돌려 같은 값을 얻었다. `oneway_loop` (6295143, 0, 0), 끝까지. 기본 지도 묶음 켬 (5074536, 0, 0), 끝까지. `oneway_shortcut` 묶음 켬 (15871223, 0, 0), 끝까지. `oneway_shortcut`과 기본 지도의 묶음 끔은 교착이 나오는 것까지 확인했다(첫 교착에서 멈춤). 표는 작업 기록 3번 |
| 전수 검사, 특징 지도(구역 14개, 지점 15개) | 우회 전 코드(`4f7cdde` 무렵, 작업 기록 "특징 지도"): 로봇 2대 bundle on (48282, 0, 0), off (47378, 42, 37), 둘 다 끝까지. 로봇 3대 bundle on은 20만 상태 상한 탐색(201140 상태, 교착 0). **결정 8의 "특징 지도 3대 끝까지"는 아직 못 했다**(아래 우회 줄의 3대도 30만 상한) |
| 우회(파트 5, 특징 지도, `df2a02b`) | 로봇 2대 bundle on, 끝까지. (상태, 교착, 기아, 서 있는 보유자 뒤에서 기다린 (상태, 로봇) 쌍, 그중 돌 길이 있던 쌍) 우회 끔 (48282, 0, 0, 316, 76), 켬 (53720, 0, 0, 276, 16). 막힘 사건(`--blocks`) 끔 (356132, 0, 0, 12458, 4444), 켬 (1198886, 0, 0, 25236, 7434). 켬은 경로와 왕복 금지 목록이 상태에 들어가 상태가 늘어서, 막힘 사건이 있으면 쌍의 절대 수가 늘고 상태 하나당 쌍은 준다(0.035→0.021, 0.0125→0.0062). 한 상태에서 여러 로봇이 기다리면 여러 번 센다(3대). 로봇 3대 막힘 사건은 30만 상태 상한 탐색이다(끔 324889 상태, 교착 0, 36756, 586. 켬 330583 상태, 교착 0, 36515, 202. 기아는 끝까지 돌아야 잰다). 기본 지도와 oneway 지도 다섯 경우는 우회 켬(파일 값)에서도 위 숫자와 같다. **한계**: 모형은 시계가 0이고 상태를 되살릴 때 `wait_zone`을 지운다. 그래서 구역을 기다리는 보유자의 벌점은 늘 0이고, "기다린 시간이 늘어 우회하는" 경우는 탐색하지 않는다. 우회는 스테이션·대기 칸에 선 보유자(추정 30초), 막힌 레인, 무한 보유자 때만 일어난다. 막힘 표시와 취소 결과는 한 사건이고 줄 서기 `BLOCKED`는 없다. 기아 0은 로봇마다, 그 로봇이 갈 곳이 남은 어느 상태에서든 cancel이 아닌 사건 순서로 그 로봇이 다시 움직이는 상태에 닿을 수 있다는 뜻이다. 모든 로봇이 함께 끝내는 사건 순서 하나가 있는지는 보지 않고, 모든 일정이 그 길을 간다는 뜻도 아니다. 교착·기아 없음의 보장으로 읽지 않는다. 시나리오와 표는 작업 기록 파트 5 |
| 기록 재생 | 전체 시험 안의 통합 시나리오는 모두 기록과 같은 명령이다(`33f01d7`). `df2a02b`: 9개 기록과 옛 기록 6개(`scen_latest2`) 모두 같다. 파트 5.5 뒤에는 옛 `operator_recovery` 기록만 1137번째 줄에서 다르다. 격리로 새 내부 명령 `RobotIsolated`가 나오기 때문이고 예상한 차이다 |
| FMS 재시작 | 죽인 뒤 세 대 모두 1.5 s 안에 정지(벽시계, 5회 측정, 최대 1.49 s. 3회는 5번 리뷰를 반영한 코드로 `fa94eb6`에 커밋, 2회는 6번 리뷰를 반영한 코드로 `1f56911`에 커밋). 새 FMS는 전원 보고부터 다시 시작. `b6d75d9`의 전체 시험에서도 통과했지만 이 시험의 판정선은 1.5 s에 여유 1 s를 더한 값이고 그 실행의 초는 적어 두지 않았다 |
| 압박 시험 | 커밋 전 작업 트리. d 0.2 표는 `fa94eb6`에, 나머지 표는 `1f56911`에 7번 변경을 얹은 코드다(`78011a2`에 커밋). 그 뒤 다시 돌리지 않았다. d 0.2에서 τ 0.5, ε 0.05까지 전 조합 조기 해제 0, near 0, 교착 0(조합마다 시드 1개). d 0.17, ε 0.05에서 심판 기준 위반 1회. d ≤ 2r은 시작 거부. 표는 작업 기록 7번 |

모델이 다루지 않는 것은 작업 기록 3번에 있다. 전수 검사는 추상 모델이고, 재시작의 숫자는 즉시 서는 `FakePinky`의 값이며, 압박 시험의 near는 앞 로봇이 d 근처에 서는 경우를 만들지 않았다.

`08_무엇을_개발했나_설명`의 시험 527건과 시나리오 6개 결과는 이전 구조에서 얻은 값이다. 지금 구조의 결과로 인용하지 않는다.

남은 일:
1. 실제 지도로 레인 그래프를 쓰고 전수 검사와 시나리오를 다시 돌린다.
2. 실물 핑키로 ε, τ를 재서 d를 정한다.
3. FollowPath 방식(계약 3절 7번), 이동 성공의 도착 허용치, heartbeat 계약을 FSM 담당과 확인한다.
4. 가짜 workcell과 심판을 양팔 동시 계획 기준으로 고칠지 팔 담당과 정한다(`arm_planner`, SR-22).
5. 특징 지도 로봇 3대 끝까지 탐색(지금은 30만 상태 상한까지).
7. 도착 허용과 해제 정책(13절, 사용자 결정 대기).
6. 특징 지도 시나리오 3개를 통합 시험 목록에 넣을지(지금은 전체 시험 시간 때문에 따로 돌린다).

## 12. Confluence SR과 다른 점 (819201 v2와 2026-11-04 대조)
기준 문서는 Confluence이고, 이 작업에서 Confluence는 고치지 않는다. 아래 항목은 어느 쪽으로 맞출지 정한 뒤 이 표에서 지운다.

| SR | SR 내용 | 이 문서와 코드 | 정할 것 |
|---|---|---|---|
| SR-22 (MUST) | 상품 놓인 곳과 바구니 위에는 각각 한 번에 한 팔만 들어간다 | 2절은 양팔 동시 궤적 계획을 쓰고 잠금 방식을 폐기했다(2026-11-04 결정). SR에는 반영되지 않았다 | SR을 고칠지, 한 팔씩 들어가는 방식으로 돌아갈지. 2026-11-05에 팀과 상의한다 |
| SR-13 (MUST) | 핑키 1대를 양보시켜 길 막힘을 자동으로 풀고, 안 되면 관리자에게 알린다 | 5절의 `DEADLOCK_RETREAT`는 아직 들어가지 않은 구역의 예약만 포기한다. 들어간 구역에서 물러나는 동작은 없다 | 이것으로 충족한다고 볼지, 양보 주행을 만들지. 구현 전이라 일단 둔다(2026-11-04) |
| SR-03 (MUST) | 주문 상태: 대기, 배정, 담는 중, 운반 중, 출고 완료, 보류, 실패 | 5-1절: PENDING, BOUND, FILLING, TO_OUTBOUND, SWAPPING, SHIPPED, PARTIAL, HELD. "실패"가 없고, SWAPPING과 PARTIAL이 SR에 없다 | 상태 대응표와 "실패"의 뜻 |
| SR-17 (MUST) | 핑키 상태: 대기, 이동, 도킹, 담기 대기, 출고 대기, 오류 + 원인 | `/fleet/robots`(제안)로 전부 나타낼 수 있다. 위에서부터 먼저 맞는 것을 쓴다: `fault` ≠ NONE이면 오류 + `fault_reason`, `task_type` = FILL이면 담기 대기, SWAP이면 출고 대기, `fsm_state` = DOCKING이면 도킹, MOVING이면 이동, 나머지는 대기. 이 규칙의 구현은 화면 몫이다 | 메시지를 팀이 받아들일지 |
| SR-36 (MUST) | 관리 프로그램과 별개의 비상 정지 | 이 문서와 `contract.yaml`에 경로가 없다. STOP은 화면, central, FMS를 거친다. `/fleet/heartbeat`는 FMS나 통신이 끊겼을 때 로봇이 혼자 서는 장치이고 비상 정지 경로를 대신하지 않는다 | 경로와 듣는 쪽 |
| SR-27 (MUST) | 상품의 종류, 위치, 수량과 트레이·바구니 위치 인식 | `/{arm}/vision/items`는 arm1, arm2의 상품만 다룬다. 트레이·바구니 인식과 arm3의 인식 인터페이스가 없다 | 어디서 어떤 토픽으로 낼지 |
| SR-21 (MUST) | 핑키가 조금 어긋나게 서도 위치를 맞춰 담는다 | dispatch 결과에 도킹 오차가 있지만 `FillCmd`와 FillBasket goal에는 전달할 필드가 없다 | 오차를 넘길지, 카메라로 보정할지 |
| SR-31 (MUST) | 지도 위 핑키 위치 표시 | `/fleet/robots`의 x, y, yaw(제안, 1 Hz, 마지막 보고 값) | 메시지를 팀이 받아들일지, 1 Hz로 충분한지 |
| SR-15 (SHOULD), UR-10 | 배터리가 낮으면 대기 구역으로 돌아가 충전 | `RobotState.battery`를 `/fleet/robots`로 보여 주기만 하고 `traffic`에 판단이 없다 | 넣을지 |
| SR-09, SR-20, SR-23, SR-25, SR-28 (SHOULD) | 주문 취소, 사람 접근 대응, 작업 팔 거리 감시, 빈 바구니·출고 위치 관리, 담긴 개수 확인 | 이 문서에 전용 규칙과 인터페이스가 없다. `put_empty`와 `placed`는 있지만, 빈 바구니의 보관 수량 관리나 카메라로 하는 개수 확인은 아니다 | 어디까지 넣을지 |

## 13. 아직 정하지 않은 것
정한 것(11-04~05, 다시 묻지 않는다):
- **Open-RMF는 도입하지 않는다.** 계층 나누기와 용어만 참고한다. 이유: 코어가 orchestrator·traffic 자리를 둘 다 차지해 팀 구조가 통째로 바뀌고, 시간 궤적 협상은 Wi-Fi 지연·정지 오차에 민감하며 전수 검사 같은 증명을 못 주고, 8주 일정에 위험.
- 시간·공간 예약, MAPF, 우선순위 최적화는 하지 않는다.
- "어디에 뭐가 있다"(인식 결과, 상태, 명령)는 ROS, 픽셀(영상)은 ROS 밖이다. 집기용 사진은 HTTP 스냅샷 한 장, 화면용은 MJPEG over HTTP. 영상을 DDS로 Wi-Fi에 흘리지 않는다.
- **핑키 계약(11-06, 사용자 지시)**: 관제가 핑키에 요구하는 통신 규격과 동작은 `docs/핑키_계약.md`에 요구사항으로 정했다. 제안으로 두지 않는다. 선점, 취소, 실패 반환, 도착 0.15 m, heartbeat 정지, `LOCALIZING`, `BLOCKED` 판정과 관제의 막힘 처리(5초 뒤 레인 표시, 60초 뒤 ERROR, 줄 서기 제외)를 담는다. `BLOCKED`, `LOCALIZING`, `HEARTBEAT_LOST`는 지시서 4번에서 정의 파일과 관제 코드에 넣었다. `MoveTo.path`는 지시서 7번에서 넣었다.
- **우회(11-06 지시서 5번)**: 다음 구역을 청할 때만 비용(레인 길이 + 서 있는 보유자의 예상 대기 × 속도, 막힌 레인은 못 지남)으로 다시 고른다. 서 있음과 달림, 속도 0.2 m/s, `DETOUR`(INFO)는 5절에 정했다. 30초 추정과 0.2 m/s는 실물에서 다시 본다.
- **이탈 e(11-06 지시서 3번)**: 해제 조건에 넣지 않되 이탈 중에는 해제를 멈춘다. 조건에 넣으면 레인이 나란한 지도에서 로봇이 다음 구역을 청하지 못할 수 있어서다. 이탈한 로봇은 세우고 구역을 쥔다(5절). `lane_error_warn_m`과 `off_lane_timeout_s`는 실측 e를 본 뒤 다시 정한다.
- **치운 로봇을 다시 투입하는 절차(11-06 지시서 6번)**: 관리자가 `/fleet/reinstate`(robot_id, basket ∈ {EMPTY, NONE}, note)를 부른다(4절). 로봇은 다음 위치 보고부터 시작 때와 같은 절차로 맵에 오르고(이동·도킹 중이거나 배치 충돌이면 기다림), 그동안 다른 로봇은 멈추지 않는다. 단 남이 쥔 구역에 보고되면 그 주인만 세우고, 둘 다 제자리면 관리자가 옮기거나 다시 치워야 풀린다(5절). 바구니는 관리자가 본 값이고, NONE이면 주문 전에 출고 구역에서 빈 바구니를 받는다(5절). 서비스 이름과 타입은 0.3-draft 관제 요구(팀 합의 전)이고 중앙 담당과 확인한다.
- **레인 준수 방식(11-06 지시서 결정 2, 7번)**: FollowPath로 정했다. 관제가 레인 폴리라인을 `MoveTo.path`로 보내고 핑키가 그 선을 따른다. "관제가 잡은 공간 = 실제 주행선"을 노린다. `point` 모드는 호환·비교용으로만 남긴다. FSM·Nav2 담당과 확인 전이다.

  | 방식 | 장점 | 단점 |
  |---|---|---|
  | 점 목표(`point`) | 핑키는 NavigateToPose만 쓰면 된다. 지금 FSM과 가장 가깝다 | 꺾일 때마다 서고 다음 목표를 기다린다. 점 사이는 Nav2 플래너가 길을 정하므로 레인을 벗어날 수 있다. BT 복구(Spin, BackUp)가 잡지 않은 공간으로 움직일 수 있다 |
  | 점 목표 + Nav2 keepout 필터 + BT 복구 축소 | 지도 마스크로 금지 구역을 막는다 | keepout은 방향을 표현하지 못해 2차선 대로에서 마주 오는 레인으로 넘어갈 수 있다. 좁은 통로에서 마스크와 팽창 반경 튜닝 부담이 크다. 마스크를 잠금에 맞춰 바꾸기 어렵다 |
  | FollowPath(`path`, 채택) | 관제가 잡은 공간과 실제 주행선을 같게 하려는 방식이다. 꺾임에서 서지 않게 한다. 플래너와 BT 복구를 쓰지 않는다 | 핑키 FSM이 FollowPath 클라이언트, 끝 근처 재전송, 도착 판정을 새로 가져야 한다. 선 위 물체는 돌아가지 않고 선다(BLOCKED, 계약 4절) |

- **FSM 담당에게 넘길 것(파트 7)**: 옮길 코드는 `~/roscamp_ws/sim/pinky_multi_sim`에 있다. 경로 만들기(`free_path.py`의 `poses_along`: 선을 따라 촘촘히 점을 찍고 방향을 선에 맞춘다, `missions.py`의 `Driver.drive`: FollowPath goal), 끝 근처 재전송(`traffic.py`의 `REFRESH_WITHIN_M` 0.15 m, `RESEND_EVERY_S` 0.5 s: 끝 근처에서 그날의 AMCL로 끝을 다시 재게 한다), 도착 판정(`traffic.py`의 `goal_done`: 성공이어도 끝에서 `ARRIVED_M` 0.15 m 안일 때만 도착). RPP `rotate_to_heading_min_angle`은 0.25(`nav2_overrides.yaml`)를 유지한다. 지시서에 따르면 0.8로 올리면 충돌 경고가 8건에서 717건으로 늘었다(이 작업에서 다시 재지 않았다). 관제가 보내는 선은 지점만 담으므로 촘촘한 점 찍기는 핑키가 한다.

정하지 않은 것:
- **도착 허용과 해제(사용자 결정 대기, 파트 7 리뷰)**: 관제는 `arrival_tolerance_m`(0.15 m) 안이면 도착으로 본다. 그만큼 못 미쳐 서면 해제 조건(s > d, 다른 바닥 선에서 d)이 안 될 수 있다. 그러면 뒤 구역이 안 풀리고, 현재 구역 말고 쥔 것이 있으면 다음 구역을 청하지 않아(`controller.py`의 `_request_next_zone`) 조용히 멈춘다. 0.15 m 못 미쳐 서면 안 풀리는 레인 수(de35745 지도, 파트 6은 지도를 바꾸지 않았다, 주 세션 계산): 기본 지도 0, 두 2차선 지도 0, feature_map 6(W→E0, E1→E2, E2→O, O→F0, F0→F1, F2→W), oneway_loop 13, oneway_shortcut 13. 0.10 m이면 모든 지도에서 0이다. 최종 지도(`a12b159`)에서 다시 재도 같다(주 세션, `stopshort.py`). 안: (1, 추천) 계약의 도착 허용을 0.10 m로 좁히고 시작 검사가 L − 허용값에서 해제 조건을 보게 한다. 시뮬 영향 없음. (2) 구간을 0.4 m 이상으로 늘린다. 특징 지도와 파트 5 검증을 다시 해야 한다. (3) 그대로 두고 감시 이벤트만 더한다. 멈춤은 보이지만 사람이 풀어야 한다.
- **FMS 구조와 담당**: v17 구조 자체가 팀 합의 전이다. FMS 담당은 정하지 않았다. 제안은 v17의 core와 pinky(코드의 `traffic`과 주문 진행)를 핑키 관제 담당이, arm을 팔 담당이 맡는 것이다.
- **새로 제안한 바깥 인터페이스**: `/fleet/robots`의 이름과 필드. 핑키 쪽(`/fleet/heartbeat` 포함)은 위 "핑키 계약"으로 정했다.
- **시작 정책**: 지도에 못 올리는 로봇(`PLACEMENT_CONFLICT`)이 있으면 전원이 기다리는 것(지금 코드)이 맞는지.
- **실측 값**: ε, τ, v를 실물 핑키로 재서 d를 정한다(작업 기록 7번의 표). 실제 지도의 레인 그래프는 2·3번 도구에 데이터만 넣으면 된다.
- STOP/PAUSE는 FMS가 OrderBoard의 run_mode로 받는다. SR-34의 두 정지 방식은 PAUSE(하던 동작을 끝내고 멈춤)와 STOP(취소하고 바로 멈춤)에 대응한다. 받아들일지는 중앙 담당과 확인한다.
- 비상 정지의 경로와 어느 노드가 듣는지(SR-36): 화면의 STOP과 달리 FMS에 기대지 않아야 한다.
- 화면에서 central로 가는 `confirm_clear`의 이름과 타입: 4절에 없다. 중앙 담당과 정한다. 화면과 central 사이를 HTTP로 할지(제안)와 실시간 상태를 띄우는 방식도 함께 정한다.
- FMS 재시작 뒤 진행 중이던 주문과 바구니를 어떻게 할지(7절).
- arm_planner와 workcell 구조, SR-22(한 번에 한 팔): 팔 담당. FMS 쪽은 `fill_basket`, `swap_basket` 클라이언트만 유지한다.
- arm3의 인식(SR-27): v17 HW 그림에는 arm3 카메라가 있지만 `arm_vision`과 `/{arm}/vision/items`는 arm1, arm2뿐이다.
- Vision 서버(v17 제안)를 둘지, 어느 PC에서 돌릴지. DB 종류(제안: SQLite, central만 읽고 쓴다).
- workcell이 arm1, arm2 중 어느 팔 PC에서 도는지: 팔 파트가 정한다.
- JetCobot의 Raspberry Pi 5를 어떻게 쓸지: 지금 구성은 팔을 팔 PC로 제어한다. 11-05 권장은 제트코봇 3대가 고정 장비이므로 Pi 5를 유선으로 잇는 것이고, 그러면 Wi-Fi는 핑키 3대만 쓴다. 10절의 역할 배정은 확인이 필요하다.
- 공유기 기종과 위치, 관제 PC와 팔 PC의 사양.
- 상품 영역의 정확한 위치, 출고 선반, 빈 바구니 공급 방법.
- 지도: `09_맵_아키텍처_결정_항목.txt`의 지점(그 문서의 노드), 레인, 잠금 단위, 도크 접근, 트레이 포함 폭, 팔 위치.

## 14. 다른 문서와 다른 점 (이 문서들은 이번에 고치지 않았다)
| 문서 | 이 문서와 다른 내용 |
|---|---|
| `01_team_roles` | 관제가 주문을 묶고 담기와 교체 목표를 보낸다(FMS 구조에서는 이 부분이 맞는다). FMS 담당이 없다. 작업공간 서버가 한 번에 1대만 넣는다. 팔 명령은 각 Pi에서 돈다고 적었다(10절은 팔 PC). |
| `02_프로젝트_설계_초안_GG` | 관제가 `fill_basket`, `swap_basket`을 직접 호출한다(FMS 구조에서는 이 부분이 맞는다). UR 2785281과 SR 62개를 기준으로 삼는다. |
| `03_fake_로봇_선개발_계획` | 팔 2대는 한 번에 한 팔만 PLACING이다. |
| `04_GitHub_저장소_구성안` | workcell 패키지 자리가 정해지지 않았다. |
| `08_무엇을_개발했나_설명` | 이전 구조의 코드와 시험 결과를 설명한다(11절). SR 62개 기준의 번호를 쓴다. |
| 저장소 `README` | 양팔 규칙(`dual_arm_overlaps`)은 이전 정의를 따른다(6절, 11절). 나머지는 이 문서와 같다. |
| `05_관제_시뮬레이션_레인.html` | 조기 해제와 근접의 정의는 같다(`safety_meter.py`). 지도와 숫자는 그 시뮬의 것이다. |
| `시스템_아키텍처.html` v18(2026-11-06, Drive 수정 2026-11-05 15:23 UTC) | 이 문서와 같은 계층을 쓴다. FMS 안을 core, pinky, arm 세 모듈로 그렸다(0절의 대응). workcell은 FMS에 합치지 않고 FMS는 `/workcell/fill_basket` 클라이언트만 둔다(코드와 같다). 코드 이름을 0.5 전 이름(`orchestrator_core`, `traffic_core`, `FmsCore`, `fms_node`)으로 적는다. MAP 부분은 이전 격자 지도(구역 16개, 3대 전수 149,238 상태)이고 3절과 다르다. 11-06 결정(머리 줄)은 들어 있지 않다(FollowPath가 나오지 않는다). |
| `교통파트_지금할일_2026-11-05` | 지도 회의에 요소 하나만 넣자고 했고 기본 지도에는 둘(양방향 구간, 교차)이 있다(3절). `/fleet/assignments`에 필드를 더하자고 했고 코드는 `/fleet/robots`를 따로 둔다(지시서 4번). 코드 이름 `fleet_core`를 바꾸자고 했고 그대로 뒀다(11절). 표의 `(149238, 0, 0)`은 이전 지도의 값이다. |
| Drive `Pinky Pro 3대 시뮬레이션 관제 정리 (2026-11-05)`, 코드 `~/roscamp_ws/sim/pinky_multi_sim` | Gazebo 3대 시뮬의 관제다. 별개 코드이고 커밋되지 않았으며 fleet_core와 통합하지 않는다. 자유 선 경로와 연속 공간 예약을 쓴다. 규칙만 옮겨 적는 출처다(아래). |

**Gazebo 시뮬(pinky_multi_sim)과의 관계** (11-06 결정 1, 2. 그 폴더는 읽기만 했다)

옮긴 규칙:

| 규칙 | pinky_multi_sim | fleet_core |
|---|---|---|
| 멈춤 감시 | `traffic.py` `_watch_for_stall`. `STALL_S` 6, `MOVED_M` 0.02 | `traffic/controller.py` `_watch_stall`. `stall_s` 6, `stall_move_m` 0.05(위치 잡음 때문에 올림). 지수 평균(`stall_smooth_s`)과 도킹용 `dock_stall_s` 15는 더한 것 |
| 도착 판정 | `goal_done`. 성공 보고라도 선 끝이고 끝점 `ARRIVED_M` 0.15 안이어야 도착 | `_judge_arrival`. `arrival_tolerance_m` 0.15, 보고가 늦을 때 `arrival_settle_s` 1 |
| 대기-vs-우회 | `_blocked`, `_expected_wait`. 앞 로봇이 작업 중이면 남은 시간, 주행 중이면 기다린 시간, 고장이면 무한. 기다리는 사슬의 맨 앞을 본다. 속도 0.25 m/s | `traffic/detour.py`. 스테이션·대기 칸은 `station_wait_estimate_s` 30(교통은 팔 작업 시간을 모른다), 구역을 기다리면 기다린 시간(사슬은 보지 않는다), `detour_speed_mps` 0.2. 다음 구역을 청할 때만 다시 고른다 |
| 물체 앞 정지 | 관제가 라이다로 물체를 찾고, 돌 길이 없으면 `ABANDON_S` 15초 뒤 임무를 포기 | 로봇이 `BLOCKED`를 보고하면 5초에 레인 표시, 60초에 ERROR와 자동 격리(`traffic/blocking.py`) |

옮기지 않은 것: sim의 "서로 기다리면 늦게 시작한 임무가 비킨다"(`_yields_in`, `_give_way`). 교착은 지금 방식(`DEADLOCK_RETREAT`)이다.

아직 옮기지 않은 것(FSM 담당 몫): 촘촘한 점 찍기(`poses_along`), 끝 근처 재전송(`REFRESH_WITHIN_M` 0.15 안이면 0.5초마다), FollowPath 호출(`missions.py`). 관제는 파트 7부터 지점만 담은 선을 `MoveTo.path`로 보낸다(13절 "FSM 담당에게 넘길 것").

옮기지 않는 것(지시서 9절): obstacle_watch의 라이다 합치기(감지는 로봇이 한다), 자유 선 경로 + 연속 공간 예약(`free_path.py`, `_extend`), Gazebo를 CI나 시나리오에 넣는 것. fleet_core와 `.github/workflows/ci.yml`에 해당 코드가 없다.
