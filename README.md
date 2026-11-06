# GG 주문 피킹 관제 프로토타입

심화 16기 2팀 GG 프로젝트의 관제를 실물 없이 개발하고 시험하기 위한 프로토타입입니다. 주문이 들어오면 핑키가 대기 구간에서 출발하고, 공유 작업공간의 로봇팔 2대가 핑키 뒤 트레이의 바구니에 주문 상품을 담습니다. 핑키가 출고 구역으로 가면 로봇팔 1대가 바구니째 꺼내고 빈 바구니를 트레이에 넣습니다. 그다음 핑키는 대기 구간으로 돌아가거나, 주문이 밀려 있으면 바로 작업공간으로 갑니다.

**이 저장소의 모든 결과는 가짜 로봇으로 얻은 것입니다.** 실물 핑키, 실물 로봇팔, 실제 카메라로 시험한 적이 없으므로 시간, 정밀도, 그리퍼 성능에 대해서는 아무것도 말해 주지 않습니다.

폴더 구조는 `04_GitHub_저장소_구성안.pdf`를 따릅니다. 인터페이스, 지도, 수치는 아직 팀 합의 전의 초안이며 작업공간 위치도 미정입니다.

FMS(fleet manager, RCS)는 핑키 플릿에 일을 나눠 주고(배분), 길을 정리하고(교통), 스테이션에 작업을 요청합니다(핸드셰이크). 팔의 동작 자체는 스테이션이 합니다. FMS는 프로세스 하나이고, 안에 `orders`와 `traffic` 두 모듈이 있습니다.

### 모듈 의존성 (누가 누구를 import하나)

```mermaid
flowchart LR
  subgraph fleet["gg_fleet (ROS 노드)"]
    fmnode["fleet_manager_node"]
    adapters["adapters"]
    tracker["goal_tracker"]
  end
  subgraph core["gg_fleet_core (ROS 없음)"]
    manager["manager<br/>FleetManager"]
    subgraph orders["orders"]
      orch["orchestrator<br/>OrderOrchestrator"]
    end
    subgraph traffic["traffic"]
      ctrl["controller<br/>TrafficController"]
      mission["mission"]
      recovery["recovery"]
      zones["zones<br/>ZoneLocks"]
      selector["selector<br/>RobotSelector"]
      deadlock["deadlock"]
      lanemap["lane_map<br/>LaneMap"]
    end
    subgraph common["common"]
      commands["commands"]
      params["params"]
      contract["contract"]
    end
  end
  subgraph sim["gg_sim (가짜 노드, 심판)"]
    fakes["fake_*_node<br/>scenario_monitor_node"]
  end
  fmnode --> adapters & tracker & manager & lanemap & contract & params & commands
  adapters --> commands
  manager --> orch & ctrl & mission & commands
  orch --> commands
  ctrl --> mission & recovery & zones & selector & commands
  recovery --> mission & deadlock & commands
  fakes --> contract & lanemap
```

`orders`와 `traffic`은 서로 import하지 않고, `manager`가 `common/commands.py`의 명령으로 둘 사이를 잇습니다. `pc/gg_fleet_core/test/test_fms_boundary.py`가 이 경계를 검사합니다. `traffic`은 주문과 바구니를 모릅니다.

### ROS 노드 지도 (컴퓨터별)

```mermaid
flowchart TB
  web["화면 (브라우저)"]
  subgraph pcbox["관제 PC (유선)"]
    central["central<br/>중앙 담당, 지금은 가짜"]
    fm["fleet_manager<br/>gg_fleet, 안에 orders + traffic"]
  end
  subgraph pinky["핑키 Pi ×3 (Wi-Fi, /pinky1~3)"]
    fsm["pinky_fsm"]
    nav["Nav2 + AMCL"]
    bring["pinky_bringup"]
  end
  subgraph armw["팔 PC: 작업공간 (유선)"]
    wc["workcell / arm_planner"]
    ac["arm_control ×2"]
    av["arm_vision ×2"]
  end
  subgraph armo["팔 PC: 출고 (유선)"]
    a3["arm_control (arm3)"]
  end
  web -. "HTTP (제안)" .- central
  central -- "/central/orders" --> fm
  fm -. "srv /central/report_order" .-> central
  fm -- "/central/events, /fleet/robots, /fleet/assignments, /fleet/zones" --> central
  central -. "srv /fleet/confirm_clear" .-> fm
  fm == "/pinkyN/move_to, /pinkyN/dock" ==> fsm
  fm -- "/fleet/heartbeat (제안)" --> fsm
  fsm -- "/pinkyN/state" --> fm
  fsm == "navigate_to_pose" ==> nav
  nav -- "/cmd_vel" --> bring
  bring -- "/scan, /odom" --> nav
  fm == "/workcell/fill_basket" ==> wc
  fm == "/arm3/swap_basket" ==> a3
  av -- "/armN/vision/items" --> wc
  wc == "follow_joint_trajectory" ==> ac
  ac -- "/arm1/state, /arm2/state" --> central
  a3 -- "/arm3/state" --> central
```

실선은 토픽, 점선은 서비스, 굵은 선은 액션입니다. 시험 때는 같은 인터페이스를 내는 가짜 노드(`gg_sim`의 `fake_pinky_node` ×3, `fake_workcell_node`, `fake_arm_node`, `fake_vision_node`, `fake_central_node`)와 심판 `scenario_monitor_node`, `sim_clock_node`가 관제 PC에서 대신 뜹니다. 11-06 개선 지시서의 제안(`/fleet/reinstate`, `MoveTo`의 `path`, 상태 값 `BLOCKED`, Nav2 `FollowPath`)은 구현되는 파트에서 이 그림에 더합니다. 이름, 타입, QoS의 원본은 `interfaces/gg_interfaces/contract.yaml`입니다.

## 폴더

| 폴더 | 내용 | 상태 |
|---|---|---|
| `interfaces/gg_interfaces` | 메시지, 액션, 서비스, `contract.yaml`(이름, 타입, QoS의 단일 원본) | 초안 |
| `pc/gg_fleet_core` | 관제 판단 로직. ROS를 쓰지 않는 순수 Python과 pytest. `manager.py`(`FleetManager`), `orders/`(주문), `traffic/`(교통), `common/`(명령, 파라미터, 계약), `data/`(기본 지도와 파라미터) | 동작 |
| `pc/gg_fleet` | `gg_fleet_core`를 ROS에 연결하는 노드. 실행 파일 `fleet_manager_node`, 노드 이름 `fleet_manager` | 동작 |
| `sim/gg_sim`, `sim/gg_sim/scenarios` | 가짜 핑키, 작업공간 서버, 출고 팔, 비전, 중앙, 시계, 심판 노드와 시나리오 | 동작 |
| `sim/gg_sim/maps` | 비교용 일방통행 지도 2개. 기본 지도는 `pc/gg_fleet_core/gg_fleet_core/data/lane_map.yaml` | 동작 |
| `tools` | 전수 검사(`model_check.py`), 기록 재생(`replay.py`), 지연·잡음 압박 시험(`pressure_sweep.py`), 격자 지도 변환(`convert_grid_map.py`) | 동작 |
| `bringup/gg_bringup` | `pc.launch.py` | 동작 |
| `scripts/check_interfaces.py` | 실행 중인 시스템을 `contract.yaml`과 비교 | 동작 |
| `pc/central`, `pinky/*`, `arm/*` | 담당자가 채울 빈 패키지 | 비어 있음 |

패키지 이름은 2026-11-06에 바꿨습니다(`fleet_core` → `gg_fleet_core`, `fleet_manager` → `gg_fleet`, `fleet_interfaces` → `gg_interfaces`, `fake_robots` → `gg_sim`, `iponic_bringup` → `gg_bringup`). 모듈별로 바꾼 표는 `docs/worklog_2026-11-06.md`에 있습니다. 낱말은 fleet(핑키 3대), station(작업공간·출고), waypoint(지점, 레인 그래프의 꼭짓점. ROS 노드와 헷갈리지 않게 "노드"라 부르지 않습니다), lane(레인), zone(구역, 잠금 단위), dock/parking으로 씁니다.

가짜 노드는 다음과 같습니다.

| 노드 | 맡은 일 |
|---|---|
| `fake_pinky_node` | `/{pinky}/state`, `move_to`, `dock`. 고장 `comm_loss`, `nav_abort`, `stuck`, `operator_remove` |
| `fake_workcell_node` | `/workcell/fill_basket`, `/arm1/state`, `/arm2/state`. 고장 `arm_fail`, `arm.fail_rate` |
| `fake_arm_node` | arm3만. `/arm3/swap_basket`, `/arm3/state`. 고장 `swap_partial`, `arm.fail_rate` |
| `fake_vision_node` | `/arm1/vision/items`, `/arm2/vision/items`. 실제 재고를 보여 주고 집어 간 만큼 줄어듭니다 |
| `fake_central_node` | 재고 기록과 주문 목록, `/central/orders`, `submit_order`, `report_order`, `set_run_mode`, 중앙 이벤트 코드 |
| `scenario_monitor_node` | 심판. 결과 파일을 쓰고 launch를 끝내며, `operator_remove` 때 관리자 대신 `confirm_clear`를 부릅니다 |

## 실행

ROS 없이 관제 로직만 시험합니다. Windows에서도 실행할 수 있습니다.

```bash
pip install pytest pyyaml
PYTHONPATH=pc/gg_fleet_core python -m pytest pc/gg_fleet_core/test -q
```

ROS 2 Jazzy에서 전체를 빌드하고 시나리오를 실행합니다. 로봇 네트워크의 기본 도메인과 섞이지 않도록 도메인을 따로 지정합니다.

```bash
mkdir -p ~/fleet_ws/src && cp -r . ~/fleet_ws/src/roscamp-repo-2 && cd ~/fleet_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
export ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 launch gg_bringup pc.launch.py use_fake:=true scenario:=basic_orders result_dir:=/tmp/run1
cat /tmp/run1/summary.json
```

시나리오가 끝나면 launch가 스스로 종료되고, `result_dir`에 `summary.json`, `events.csv`, `zones.csv`, `robots.csv`, `releases.csv`가 남습니다. 심판은 가짜 로봇이 `/sim/truth/<로봇>`으로 발행하는 잡음 없는 위치로 판정하고, 실물 로봇은 보고된 위치로 판정합니다.

이 코드의 공식 실행 결과는 저장소 옆 폴더 `../test_results/`에 있습니다. 옛 결과는 WSL2와 CycloneDDS에서 얻은 것이고, 지금 개발 PC에는 `rmw_cyclonedds`가 없어 기본 DDS(Fast DDS)로 실행합니다. 환경이 다르므로 옛 결과를 그대로 옮겨 말하지 않습니다. CI와 시험 의존성도 Fast DDS(`rmw_fastrtps_cpp`)로 맞췄습니다. 실물 로봇에서 쓸 DDS는 Wi-Fi 비교 시험(T-10) 뒤에 정합니다. Fast DDS는 다른 프로세스의 QoS 깊이를 알려 주지 않으므로 `check_interfaces.py`는 이 경우 깊이를 검사하지 않고 신뢰성과 지속성만 검사합니다.

## 시나리오

`sim/gg_sim/scenarios/*.yaml`에 여섯 개가 있습니다. 모든 로봇은 대기 지점에서 빈 바구니를 싣고 시작합니다.

| 이름 | 내용 | 확인하는 것 |
|---|---|---|
| `basic_orders` | 주문 6건, 고장 없음 | 전부 출고, 등록 순서 유지 |
| `order_burst` | 한꺼번에 8건, 진행 중 2건 추가 | 주문이 밀려 있으면 대기 구간을 거치지 않고 바로 작업공간으로 돌아감 |
| `shared_lane_failure` | 통신 두절과 주행 실패 | 구역 잠금 유지, `COMM_LOST`, `COMM_RESTORED`, `GOAL_RETRY`, 전부 출고 |
| `stock_shortage` | 재고 0인 품목, 기록만 있고 실물 없는 품목, 용량 초과 주문 | PARTIAL과 HELD, `OUT_OF_STOCK`과 `STOCK_MISMATCH` 구분 |
| `stuck_robot` | 돌아오던 로봇이 대로(양방향 구간)에서 영구 정지 | 뒤따르는 로봇이 막히고 `COMM_LOST`, `ZONE_WAIT_LONG`이 남 |
| `operator_recovery` | 팔 고장 3회로 ERROR, 관리자가 로봇을 치움. 다음 로봇도 담는 도중 치움(팔 취소가 느림) | `ORDER_REQUEUED` 후 앞으로 되돌아가고, 세 번째 로봇이 앞 작업의 취소 결과를 기다렸다가 전부 끝냄 |

`expect` 항목은 다음과 같습니다. 숫자는 정확히 일치해야 통과합니다(`_min`으로 끝나는 것은 그 값 이상).

| 항목 | 뜻 |
|---|---|
| `all_orders_finished` | 모든 주문이 최종 보고됨(true) 또는 일부가 끝나지 못함(false) |
| `orders_shipped`, `orders_partial`, `orders_held` | `ORDER_SHIPPED`, `ORDER_PARTIAL`, `ORDER_HELD` 건수. 적지 않으면 검사하지 않습니다 |
| `orders_shipped_min` | 최소 출고 건수. 고장이 있는 시나리오는 이 값이나 `orders_shipped`가 1 이상이어야 파일을 읽을 수 있습니다 |
| `fifo_violations` | `ORDER_BOUND` 순서가 등록 순서와 다르거나 출고(SHIPPED, PARTIAL) 순서가 묶인 순서와 다른 건수. `ORDER_REQUEUED`된 주문은 다음에 묶여야 하고, 되돌림 없이 두 번 묶이면 위반입니다 |
| `dual_arm_overlaps` | arm1과 arm2가 동시에 PLACING인 횟수. 각 팔 상태의 `stamp`로 비교합니다. 출고한 주문에 두 팔이 다 필요했는데 PLACING을 못 본 팔이 있으면 실패입니다 |
| `direct_returns_min` | 출고 구역에서 대기 구간을 거치지 않고 작업공간으로 돌아간 횟수의 최솟값 |
| `parked_while_backlog` | 묶이지 않은 주문이 로봇 수 이상인데 출고 구역에서 대기 구간으로 들어간 횟수 |
| `zone_holder_max` | 한 구역에 실제로 동시에 있었던 로봇 수의 최댓값 |
| `released_without_position_check` | 로봇이 아직 구역을 벗어나지 않았는데 해제된 횟수 |
| `unheld_zone_pose` | 잠그지 않은 구역에 로봇이 있었던 횟수 |
| `timed_out` | 제한 시간까지 끝나지 못함. 기본 false이고, 일부러 끝나지 않는 시나리오만 true로 적습니다 |
| `unexpected_errors` | `events_include`, `event_counts`에 없는 코드의 ERROR 이벤트 수. 기본 0 |
| `order_results` | 주문별 최종 결과. 예: `{O-001: SHIPPED, O-003: HELD}` |
| `events_include` | 반드시 나와야 하는 이벤트 코드 |
| `event_counts` | 코드별로 정확히 나와야 하는 횟수. 예: `{OUT_OF_STOCK: 2, STOCK_MISMATCH: 2}` |

고장 형식은 다음과 같습니다.

```yaml
faults:
  - {at_s: 40, robot: pinky2, type: comm_loss, duration_s: 20}
  - {at_s: 50, robot: pinky1, type: nav_abort, count: 2}
  - {at_s: 50, robot: pinky1, type: stuck}
  - {at_s: 60, robot: pinky1, type: operator_remove}   # 코스에서 치운 뒤 심판이 confirm_clear를 부릅니다
  - {at_s: 0, arm: arm1, type: arm_fail, count: 3}     # after_items: N을 주면 N개를 담은 뒤 고장납니다
  - {at_s: 0, type: swap_partial, count: 1}            # 찬 바구니는 꺼내지만 빈 바구니를 못 놓습니다
```

## 실물로 교체

```bash
ros2 launch gg_bringup pc.launch.py use_fake:=true fake_robots:="pinky2 pinky3"   # pinky1만 실물
ros2 launch gg_bringup pc.launch.py use_fake:=true fake_arms:="arm3"              # 작업공간 서버와 비전만 실물
ros2 launch gg_bringup pc.launch.py use_fake:=true fake_arms:="arm1 arm2"         # arm3만 실물
ros2 launch gg_bringup pc.launch.py use_fake:=false                               # 전부 실물
```

`fake_arms`에서 arm1과 arm2는 같이 쓰거나 같이 뺍니다. 서버 하나(`/workcell/fill_basket`)가 두 팔을 맡기 때문이고, 하나만 쓰면 launch가 오류로 멈춥니다. 두 이름을 쓰면 가짜 작업공간 서버와 비전 2개가, arm3을 쓰면 가짜 출고 팔이 뜹니다.

## 규격이 정해지면 고칠 곳

| 정해지는 값 | 고칠 곳 | 코드 수정 |
|---|---|---|
| 토픽, 액션, 서비스 이름 | `contract.yaml`의 `name` | 없음 |
| QoS | `contract.yaml`의 `qos_profiles` | 없음 |
| 메시지 필드 | `interfaces`의 정의, `gg_fleet/adapters.py`, `sim/gg_sim`에서 그 메시지를 쓰는 노드 | 이 파일들만. 타입을 통째로 바꾸면 `fleet_manager_node.py`의 타입 지정도 고칩니다. |
| 로봇 상태 값 | `RobotState.msg`의 상수, `sim/gg_sim/gg_sim/judge.py`의 `IDLE` 비교 | 이 파일들만. `gg_fleet_core`는 이 값으로 판단하지 않습니다. |
| 팔 상태 값 | `ArmState.msg`의 상수, `judge.py`의 `PLACING` | 이 파일들만 |
| 이벤트 코드 | `gg_fleet_core`, `fake_central_node.py`, `judge.py`, 시나리오의 `events_include` | 이 파일들만 |
| 관제와 FSM의 역할 경계 | `gg_fleet`의 `fleet_manager_node.py`와 `adapters.py` | `gg_fleet_core`는 수정하지 않음 |
| 지점 좌표와 원점 | `lane_map.yaml`의 `waypoints`, `origin_m` | 없음 |
| 레인 폭과 레인 시작 판정 | `lane_map.yaml`의 `half_width_m`, `lane_entry_m` | 없음 |
| 레인과 양방향 구간 | `lane_map.yaml`의 `lanes`, `two_way`, `bundle_two_way` | 없음 |
| 잠금 단위 묶기 | `lane_map.yaml`의 `units` | 없음 |
| 해제 거리 d | `fleet_params.yaml`의 `release_distance_m` (근거식은 그 파일의 주석) | 없음 |
| 바구니 용량 | `fleet_params.yaml`의 `basket_capacity` | 없음. 관제와 가짜 중앙이 같은 파일을 읽습니다 |
| 일부만 담겨도 출고할지 | `fleet_params.yaml`의 `ship_partial` | 없음 |
| 로봇 반경, 시간 제한, 재시도 | `fleet_params.yaml` | 없음 |
| 로봇 이름과 대수 | `pc.launch.py`의 `robots`, `contract.yaml`의 `placeholders` | 없음 |
| 작업공간과 출고 구역, 대기 지점 | `lane_map.yaml`의 `stations`, `parking` | 없음 |

`lane_map.yaml`과 `fleet_params.yaml`은 `pc/gg_fleet_core/gg_fleet_core/data/`에 있습니다. 설치된 기본값 대신 다른 파일을 쓰려면 launch 인자 `map_file`, `params_file`, `contract_file`을 지정합니다.

## 패키지를 추가할 때

Python 패키지의 `setup.py`에는 `tests_require=['pytest']`를 넣어야 합니다. 이 줄이 없으면 colcon이 pytest 대신 unittest를 실행하므로 시험이 한 건도 돌지 않고, Python 3.12에서는 종료 코드 5로 CI가 실패합니다.

## 실제 FSM이 지켜야 하는 약속

- `move_to`에 새 목표가 오면 이전 목표를 멈추고 `success=false`, `reason=CANCELED`로 끝냅니다.
- 멈춰서 진행할 수 없으면 반드시 실패나 `TIMEOUT` 결과를 돌려줍니다. 관제에는 이동과 도킹 목표의 시간 제한이 없습니다.
- `move_to`의 성공은 목표 지점에서 `arrival_tolerance_m`(0.15 m) 안에 섰을 때만 돌려줍니다. 결과를 보낸 뒤 `arrival_settle_s`(1초)가 지나도록 보고된 위치가 그 안으로 들어오지 않으면 관제는 실패한 시도로 칩니다. 그동안 위치 보고가 하나도 없으면 판정하지 않고 통신 두절(`comm_timeout_s`)을 기다립니다. Nav2를 쓴다면 `xy_goal_tolerance`를 이 값보다 작게 둡니다. 이 약속과 값(추정)은 FSM·Nav2 담당과 확인 전입니다(`docs/GG_공통설계.md` 13절).
- `/pinkyN/state`를 끊기지 않게 발행합니다. 일정 시간(`comm_timeout_s`) 동안 오지 않으면 통신 두절로 판단합니다.
- 이동이나 도킹 목표를 실행하는 동안 `/fleet/heartbeat`가 `heartbeat_timeout_s`(1.5초) 넘게 오지 않으면 그 목표를 취소하고 멈춥니다. 목표 수락도 FMS가 살아 있다는 증거로 칩니다. 이 계약은 제안이고 FSM 담당과 확인 전입니다. 자세한 내용은 `docs/GG_공통설계.md` 4절에 있습니다.
- 상태 값은 `RobotState.STATE_*` 상수를 씁니다.
- `dock_id`는 스테이션 이름(`WORK`, `OUTBOUND`)입니다.

## 실제 작업공간 서버와 출고 팔이 지켜야 하는 약속

작업공간 서버 `/workcell/fill_basket`(FillBasket):

- `success=true`는 **서버가 시도를 끝냈다**는 뜻입니다. 재고가 없어 못 담은 품목이 있어도 true이고, 그 품목은 `missing`에 넣습니다.
- `success=false`는 팔이 실패했다는 뜻입니다. `placed`에는 **실제로 바구니에 들어간 것**만 넣습니다. 관제는 이 값을 믿고 남은 품목만 다시 요청합니다. 이 경우 `missing`은 비워 둡니다(재시도에서 다시 계산합니다).
- 취소되면 `success=false`, `reason=CANCELED`로, 그때까지 담은 것을 `placed`에 넣어 돌려줍니다. 관제가 STOP으로 요청한 취소는 재시도 횟수에 넣지 않고, 서버가 스스로 취소한 것은 실패로 셉니다. 결과가 올 때까지 관제가 취소를 다시 보내므로 같은 취소가 여러 번 올 수 있습니다.
- 재시도 요청의 `items`에는 아직 담지 못한 것만 들어 있습니다.
- **arm1과 arm2가 동시에 PLACING이면 안 됩니다.** 두 팔이 함께 집는 것은 되지만, 바구니 공간에는 한 번에 한 대만 들어갑니다. 조율은 서버 내부 책임이고 관제는 관여하지 않습니다.
- `/arm1/state`, `/arm2/state`를 발행합니다. 심판이 `stamp`로 겹침을 판정하므로, 한 팔이 PLACING을 벗어났다는 상태의 `stamp`가 다른 팔의 PLACING 시작 `stamp`보다 늦으면 안 됩니다.
- 실패나 거절 뒤 관제는 `arm_retry_wait_s`를 기다렸다가 다시 보냅니다. `confirm_clear`로 취소한 목표가 있으면 그 결과가 오거나 `arm_cancel_wait_s`가 지날 때까지 다음 목표를 보내지 않습니다. 그래도 바쁘면 거절해도 되지만 재시도 횟수에 들어갑니다.
- 시간 제한(`fill_timeout_s`) 안에 끝나지 않으면 관제가 취소를 보내고 `arm_cancel_wait_s` 동안 결과를 기다립니다(STOP으로 취소한 경우도 같습니다). 결과가 오지 않으면 ERROR(`ARM_NO_RESULT`)가 되어 관리자 확인이 필요합니다. 취소 뒤에는 결과를 꼭 돌려줍니다.

출고 팔 `/arm3/swap_basket`(SwapBasket):

- `take_full`, `put_empty`로 요청받은 단계만 합니다. 재시도에서는 남은 단계만 옵니다.
- `took_full`, `put_empty_done`은 그 단계가 **실제로 끝났는지**를 알립니다. 찬 바구니를 꺼냈지만 빈 바구니를 못 놓았다면 `success=false`, `took_full=true`, `put_empty_done=false`입니다. 관제는 `took_full`을 받는 즉시 출고로 보고합니다.
- 취소되면 `success=false`, `reason=CANCELED`로 그때까지 끝낸 단계를 알립니다.
- 시간 제한은 `swap_timeout_s`이고, 뒤의 처리는 작업공간 서버와 같습니다.

비전 `/{arm}/vision/items`(ItemArray): `kinds`와 `poses`는 물건 하나마다 원소 하나씩이고 길이가 같습니다. 관제는 이 토픽을 쓰지 않습니다.

## 알려진 약점

- 두 스테이션 사이는 대로 하나로만 다니고 대로 전체가 잠금 하나이므로, 대로에서 멈춘 로봇 1대는 그 이동을 모두 막습니다(`stuck_robot`이 이 상황입니다).
- 시나리오의 가짜 로봇은 기본값으로 목표에 정확히 서고 보고 지연도 없습니다. `stop_error_m`, `report_delay_s`를 주면 압박 시험을 할 수 있지만(`tools/pressure_sweep.py`), 실제 로봇의 오차 분포를 재현하지는 않습니다.
- `dual_arm_overlaps`는 상태 메시지로 판정합니다. 상태 토픽은 best effort이므로 메시지를 잃으면 겹침을 잘못 판정할 수 있습니다.
- 팔의 동작 시간, 재고 기록, 바구니 용량은 모두 가정입니다. 실제 바구니와 그리퍼로 잰 값이 아닙니다.
