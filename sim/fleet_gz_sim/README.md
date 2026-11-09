# fleet_gz_sim

Gazebo 시뮬레이션. launch가 둘 있다. 둘 다 이 컴퓨터 안에서만 돈다(아래 "격리").

| launch | 내용 |
|---|---|
| `gz.launch.py` | 핑키 3대 + 진짜 관제. `fleet_manager`·`pinky_fsm`·Nav2는 진짜, central·팔은 가짜(`fake_robots`). 3대 주행을 눈으로 확인하는 용도 |
| `one_pinky.launch.py` | 핑키 한 대 + Nav2(`pinky_nav_config`) + `pinky_fsm`. 관제는 띄우지 않는다(`drive_check`로 관제 흉내) |

## 준비 (한 번)

ROS 2 Jazzy가 깔린 Ubuntu 24.04(WSL2도 된다)에서 한다.

1. Gazebo, Nav2 등 apt 패키지

   ```bash
   sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-xacro \
     ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp
   ```

2. 로봇 모델: 제조사 `pinky_description`(`iponic.repos`의 pinky_pro 014a09f). 따로 빌드해 둔다.

   ```bash
   mkdir -p ~/pinky_deps_ws/src && cd ~/pinky_deps_ws/src
   git clone https://github.com/pinklab-art/pinky_pro.git && git -C pinky_pro checkout 014a09f
   source /opt/ros/jazzy/setup.bash
   cd ~/pinky_deps_ws && colcon build --packages-select pinky_description
   ```

3. 이 저장소: `pinky_description`을 source한 셸에서 빌드한다(`fleet_gz_sim`이 그것을 찾는다).

   ```bash
   mkdir -p ~/fleet_ws/src && cp -r <저장소> ~/fleet_ws/src/roscamp-repo-2 && cd ~/fleet_ws
   source /opt/ros/jazzy/setup.bash
   source ~/pinky_deps_ws/install/local_setup.bash
   colcon build --packages-up-to fleet_gz_sim
   ```

## 격리

실물과 같은 `/pinkyN/...` 이름을 쓰므로 다음이 아니면 시작하지 않는다(`fleet_gz_sim/isolation.py`). 두 launch 모두 무엇을 띄우기 전에 검사한다.

- `ROS_DOMAIN_ID` 43~101(로봇 네트워크의 6은 안 된다)
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS` 없음
- RMW는 `rmw_fastrtps_cpp`, Fast DDS가 따로 읽는 설정 없음(막는 변수와 파일은 `isolation.py`의 `FAST_DDS_VARIABLES`, `FAST_DDS_FILES`)

Gazebo 통신도 이 컴퓨터 안에 둔다(`GZ_IP=127.0.0.1`, `GZ_PARTITION`이 없으면 `fleet_gz_<도메인>`). 다른 사람이나 다른 시뮬과 같은 도메인을 쓰지 않는다.

## 핑키 3대 (`gz.launch.py`)

### 실행

새 터미널마다 같은 환경을 만든다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash
source ~/fleet_ws/install/local_setup.bash
export ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

```bash
# 기본: Gazebo 화면 + 3대를 한 RViz에, 시나리오 basic_orders, 도킹은 진짜 pinky_fsm(실패)
ros2 launch fleet_gz_sim gz.launch.py

# 주행 확인 추천: 24분 순환 운행, 임무를 끝까지(도킹 대역), 레인 장애물
ros2 launch fleet_gz_sim gz.launch.py scenario:=gz_roam sim_dock_stub:=true obstacles:=roam

# 창고(포트폴리오): 6 x 4 m 창고, 랙, 작업대 위 팔 3대, 21분 동안 30 s마다 주문, 레인 장애물
ros2 launch fleet_gz_sim gz.launch.py scenario:=gz_warehouse sim_dock_stub:=true obstacles:=warehouse

# 화면 없이(느린 컴퓨터, 원격)
ros2 launch fleet_gz_sim gz.launch.py scenario:=gz_roam sim_dock_stub:=true gui:=false rviz:=none
```

경기장은 시나리오의 `map`이 고른다. `map`이 없으면 `lane_arena`, `map: warehouse`면 `worlds/warehouse.sdf`, `maps/warehouse[_fine].yaml`, 관제 레인 지도 `fake_robots/maps/warehouse.yaml`. 월드와 지도는 `sites/<이름>.yaml`에서 `python3 -m fleet_gz_sim.arena --site <이름>`(소스 폴더에서)으로 만들어 저장소에 넣는다.

WSL2에서 화면이 느리면 GPU를 쓰게 한다: `export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`(그래픽 카드 이름).

뜨는 순서: Gazebo 월드(`lane_arena`) → 브리지 하나 → 로봇마다 모델, Nav2(프로세스 하나), `pinky_fsm` → 세 대가 모두 IDLE이 되면(`wait_ready`, 보통 1분 안) 관제, 가짜 작업셀·비전·출고 팔·central, `fleet_check`, 장애물 노드, 시나리오 모니터. 3분 안에 IDLE이 안 되면 launch가 끝난다. 시나리오 주문 시각은 Gazebo 시계 0부터 센다.

끝내기: Ctrl-C 한 번, 다 내려갈 때까지(10~30 s) 기다린다. 남은 것이 있는지 본다.

```bash
pgrep -af "gz sim|ruby|component_container|pinky_fsm_node|fleet_manager_node"   # 아무것도 없어야 한다
```

### 옵션

| 인자 | 기본 | 뜻 |
|---|---|---|
| `scenario` | `basic_orders` | 시나리오 이름(`fleet_gz_sim/scenarios`, 다음에 `fake_robots/scenarios`에서 찾음)이나 파일. 로봇 고장이 든 시나리오는 거부한다. `gz_roam`은 40 s마다 주문 하나씩 24분 |
| `gui` | `true` | Gazebo 화면 |
| `rviz` | `fleet` | `fleet`: 3대를 한 RViz에(로봇, 쥔 구역, 주문). 로봇 이름(`pinky2`)이면 그 로봇의 Nav2 화면. `none`이면 없음 |
| `sim_dock_stub` | `false` | 시뮬 전용 도킹 대역(아래 "도킹") |
| `obstacles` | 없음 | 레인에 상자를 놓았다 치우는 일정(`fleet_gz_sim/obstacles`의 이름이나 파일). `roam`은 300 s마다 4개, 25~30 s씩 |
| `fleet_check` | `true` | 주행 지표를 `result_dir/fleet_check.json`에(아래 "결과 확인") |
| `monitor` | `true` | 시나리오 판정. 시나리오가 끝나면 `summary.json`을 쓰고 launch를 끝낸다 |
| `result_dir` | `gz_result` | 결과 폴더(launch를 띄운 폴더 기준) |
| `fleet_params_file` | 설치된 `fleet_params.yaml` | 관제 파라미터 |
| `heartbeat_timeout_s` | `1.5` | `pinky_fsm`의 heartbeat 끊김 판정 |
| `nav_log_level` | `warn` | Nav2 로그 수준 |
| `nav_composition` | `true` | 로봇마다 Nav2 노드 9개를 프로세스 하나로. `false`면 DDS 참여자가 LOCALHOST 한도(약 32)를 넘어 pinky3이 안 보인다 |

`true`/`false` 인자에 오타가 있으면 launch가 바로 멈춘다.

### 도킹

- 기본(`sim_dock_stub:=false`): 진짜 `pinky_fsm`이 `/pinkyN/dock`을 받고 `DOCK_UNAVAILABLE`로 실패시킨다(`pinky_dock`이 아직 없다). 임무가 도크에서 멈추고 관제가 모두 세 번 시도한 뒤(`move_retry_limit`) 그 로봇을 격리한다. 주행 확인에는 이대로 충분하다.
- `sim_dock_stub:=true`: 임무를 끝까지 돌려 보고 싶을 때. `pinky_fsm`의 dock 서버를 `/pinkyN/dock_fsm`으로 비켜 두고 `sim_stand_in`이 `/pinkyN/dock`을 받는다. 로봇을 움직이지 않고, 관제가 데려다 놓은 로봇의 실제 위치가 도크 지점 0.15 m 안이면 시나리오의 `dock.duration_s` 뒤 성공, 아니면 실패로 끝낸다.
- 대역은 `use_sim_time`이 켜져 있고 격리 검사를 통과했을 때만 뜬다. Gazebo 시계(`/clock`)가 없거나 5 s(벽시계) 넘게 멈추면 도킹을 실패로 끝내고, 뜬 뒤 `use_sim_time`을 끄는 것도 거부한다. 실물 bringup(`iponic_bringup`, `pinky_fsm.launch.py`)에는 이 옵션이 없고, `pc.launch.py`에 `sim_dock_stub:=true`를 줘도 아무것도 바뀌지 않는다(`test/test_dock_stub.py`가 고정한다).

### 결과 확인

`result_dir`(기본 `./gz_result`)에 남는다.

- `fleet_check.json`: 10 s마다, 끝낼 때 한 번 더 쓴다. launch 로그에도 1분마다 한 줄(`[fleet_check]: min distance ...`).

  | 값 | 뜻 |
  |---|---|
  | `min_distance_m`, `min_distance_pair`, `min_distance_at_s` | 두 로봇 중심 사이 최소 거리(Gazebo 실제 위치, 0.1 s 안에 잰 표본끼리), 그 쌍, 시뮬 시각 |
  | `stops`, `stops_while_moving` | 0.05 m/s 넘게 달린 뒤 0.02 m/s 밑으로 0.5 s 넘게 선 횟수(제자리 회전은 정지가 아니다). 그중 로봇이 MOVING을 보고하던 중에 선 횟수 |
  | `outside_held_samples` / `zone_samples` | 구역을 쥔 로봇의 위치 표본 중 쥔 구역 바닥 밖인 수 / 관제가 구역을 낸 뒤의 전체 표본 수 |
  | `no_zone_held_samples` | 아무 구역도 쥐지 않은 동안의 표본 수(주차 칸에서 기다릴 때 등) |
  | `worst_outside_m` | 쥔 구역 바닥에서 가장 멀리 벗어난 거리 |
  | `sim_span_s`, `wall_span_s`, `real_time_factor` | 잰 시뮬 시간, 벽시계 시간, 그 비 |

- 시나리오가 끝나면(`monitor:=true`) `summary.json`(판정, 출고 수), `safety.json`, `events.csv`, `zones.csv`, `robots.csv` 등. 중간에 Ctrl-C로 끝내면 이것들은 남지 않는다. 그때는 launch 로그의 `[fleet_manager]` 줄(배정, `DOCKED`, `ROBOT_ISOLATED` 등)을 본다.
- 돌아가는 동안 다른 터미널(같은 환경)에서:

  ```bash
  ros2 topic echo /fleet/robots --once      # 로봇마다 상태, 작업, 고장
  ros2 topic echo /fleet/zones --once       # 구역마다 쥔 로봇
  ros2 run fleet_gz_sim fleet_check --ros-args -p output:=/tmp/check.json   # launch 밖에서 따로 잴 때
  ```

### 알려진 한계

- 무겁다: Gazebo + Nav2 3대 + 관제로 16코어 PC에서도 CPU를 많이 쓴다. 다른 무거운 작업과 같이 돌리지 않는다. 실시간 비율(`fleet_check.json`의 `real_time_factor`)이 1보다 많이 낮으면 Nav2 controller가 주기를 놓치고(`Control loop missed its desired rate`) 로봇이 레인에서 벗어나 관제가 세우고 격리할 수 있다(`LANE_ERROR`, `ROBOT_ISOLATED OFF_LANE`). 2026-11-08에 다른 작업과 같이 돌린 한 번(실시간 비율 0.56)에서 두 대가 이렇게 격리됐다. 숫자는 개인 브랜치 worklog에 있다.
- DDS 참여자: LOCALHOST 탐색에서는 32개 정도만 서로 보인다. 노드를 더하면 `test_launch_wiring.py`의 프로세스 수 시험이 알려 준다. 돌아가는 동안 `ros2` 명령을 여러 개 띄우면 그 수도 들어간다.

## 핑키 한 대 (`one_pinky.launch.py`)

### 준비

위 "준비"와 같다.

### 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_deps_ws/install/local_setup.bash   # pinky_description
source ~/fleet_ws/install/local_setup.bash
export ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch fleet_gz_sim one_pinky.launch.py                          # Gazebo 화면 + RViz
ros2 launch fleet_gz_sim one_pinky.launch.py gui:=false rviz:=false   # 화면 없이
```

인자: `namespace`(pinky1), `x`·`y`·`yaw`(기본 P-3 (0.35, 1.55), P-4 쪽), `gui`, `rviz`, `fsm`, `heartbeat_timeout_s`(1.5), `nav_log_level`(warn).

로봇이 IDLE이 되면(약 10 s) 다른 터미널에서 같은 환경으로 주행시킨다.

```bash
ros2 run fleet_gz_sim drive_check                         # 관제 흉내: 한 바퀴 + heartbeat 끊기, 지표 출력
ros2 run fleet_gz_sim drive_check --laps 3 --no-heartbeat-cut
ros2 run fleet_gz_sim drive_check --whole --no-heartbeat-cut  # 비교용: 주행마다 경로 전체를 한 번에
```

`drive_check`는 로봇이 서 있는 정지 지점(P-3, W, O 중 하나)에서 출발해 P-3 → W → O → P-3 순서로 돈다. 관제처럼 `move_to`를 보내고, 로봇이 구간 중간을 지날 때마다 경로를 늘린 새 `move_to`를 보낸다(안내 2.1). `/fleet/heartbeat`는 2 Hz로 보낸다. 마지막에 heartbeat와 목표를 끊고 정지까지 시간을 잰 뒤, 마지막으로 지난 지점부터 남은 길을 다시 보낸다.

지표와 2026-11-07 측정값(한 바퀴 + heartbeat 끊기):

| 지표 | 기준(안내 4절) | 화면 없이 | Gazebo 화면 + RViz |
|---|---|---|---|
| 경로 연장 직후 0.5 s 안에 `cmd_vel` 선속도·각속도가 모두 0인 채 0.1 s 넘게 | 0 | 연장 17회 중 0 | 15회 중 0 |
| 끝 0.15 m 안에 들어와 성공까지 | 6 s 넘는 일 0(R13) | 최대 2.47 s | 최대 2.61 s |
| 성공 때 실제 위치와 목표 거리 | 0.15 m 안(R4) | 최대 0.045 m | 최대 0.065 m |
| heartbeat 마지막 신호부터 `cmd_vel` 0까지 / 실제 정지까지(그동안 이동) | 1.5 s 안(R10) | 1.555 s / 1.832 s(0.353 m) | 1.565 s / 1.805 s(0.364 m) |
| `PATH_NOT_CONNECTED` | 정상 주행에서 0 | 0 | 0 |
| 보고 위치 오차 ε(AMCL) 평균 / p95 / 최대 | 작을수록 좋음(R3, 추정 0.03 m) | 0.029 / 0.050 / 0.059 m | 0.023 / 0.040 / 0.051 m |
| 실시간 비율 | | 1.0 | 0.965 |

heartbeat는 끊김 판단 시간(`heartbeat_timeout_s` 1.5 s) 뒤 다음 판단 주기(20 Hz)에 멈추므로 `cmd_vel` 0이 1.5 s를 조금 넘고, 실제 정지는 감속만큼 더 걸린다.

선속도만 0이 되는 것은 모서리 제자리 회전(R12)이다. 측정에서 선속도 0 구간 30개 중 29개가 레인 꼭짓점 0.15 m 안, 1개가 0.22 m 앞이었고, 경로를 한 번에 보낸 주행(`--whole`)도 같은 모서리에서 섰다. RPP는 모서리에서 회전과 주행을 몇 번 오가기도 한다(연장 여부와 관계없음).

RViz의 2D Goal Pose로도 보낼 수 있다. 이것은 `bt_navigator`(한 번 계획, 복구 없음)로 가고 `pinky_fsm`을 거치지 않는다.

### 구성

| 것 | 이름 |
|---|---|
| 월드 | `worlds/lane_arena.sdf`. 월드 좌표 = 지도 좌표 = `lane_map.yaml` 좌표. 레인은 바닥 그림, 벽·상자는 레인에서 0.25 m 넘게 떨어짐 |
| 지도 | `maps/lane_arena.yaml`(5 cm, costmap), `maps/lane_arena_fine.yaml`(1 cm, AMCL) |
| 로봇 모델 | `urdf/pinky_sim.urdf.xacro`: 제조사 모델 + DiffDrive, 라이다, 실제 위치. 제조사 `pinky_gz.urdf.xacro`의 램프 플러그인과 카메라는 뺐다 |
| 토픽 | `/pinky1/scan`, `/pinky1/odom`, `/pinky1/cmd_vel`, `/pinky1/joint_states`, `/pinky1/tf`, `/pinky1/tf_static`, `/clock` |
| 실제 위치 | `/pinky1/sim/truth`(`nav_msgs/Odometry`, map 좌표). 시험 도구만 읽는다 |
| 프레임 | `map`, `odom`, `base_footprint`, `rplidar_link`(접두사 없음, TF가 namespace로 나뉨) |

월드와 지도는 같은 기하(`fleet_gz_sim/arena.py`)에서 만든다. `lane_map.yaml`이나 벽을 바꾸면 다시 만든다. 시험이 저장소 파일과 생성 결과가 같은지 본다.

```bash
cd sim/fleet_gz_sim && PYTHONPATH=../../pc/fleet_core:. python3 -m fleet_gz_sim.arena
```
