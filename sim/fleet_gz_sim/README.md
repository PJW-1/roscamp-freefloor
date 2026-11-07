# fleet_gz_sim

Gazebo 시뮬레이션. launch가 둘 있다.

| launch | 상태 | 내용 |
|---|---|---|
| `one_pinky.launch.py` | 쓸 수 있음 | 핑키 한 대 + Nav2(`pinky_nav_config`) + `pinky_fsm`. 관제는 띄우지 않는다 |
| `gz.launch.py` | 자리만 있음 | 핑키 3대. `fleet_manager`·`pinky_fsm`·Nav2는 진짜, central·팔은 가짜(`fake_robots`). 지금은 격리 검사만 한다 |

## 격리

실물과 같은 `/pinkyN/...` 이름을 쓰므로 다음이 아니면 시작하지 않는다(`fleet_gz_sim/isolation.py`). 두 launch 모두 무엇을 띄우기 전에 검사한다.

- `ROS_DOMAIN_ID` 43~101
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS` 없음
- RMW는 `rmw_fastrtps_cpp`, Fast DDS가 따로 읽는 설정 없음(막는 변수와 파일은 `isolation.py`의 `FAST_DDS_VARIABLES`, `FAST_DDS_FILES`)

`one_pinky.launch.py`는 Gazebo 통신도 이 컴퓨터 안에 둔다(`GZ_IP=127.0.0.1`, `GZ_PARTITION`이 없으면 `fleet_gz_<도메인>`).

```bash
export ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch iponic_bringup sim_gz.launch.py
```

## 핑키 한 대 (`one_pinky.launch.py`)

### 준비

로봇 모델은 제조사 `pinky_description`(`iponic.repos`의 pinky_pro 014a09f)을 쓴다. 따로 빌드해 둔다.

```bash
mkdir -p ~/pinky_deps_ws/src && cd ~/pinky_deps_ws/src
git clone https://github.com/pinklab-art/pinky_pro.git && git -C pinky_pro checkout 014a09f
cd ~/pinky_deps_ws && colcon build --packages-select pinky_description
```

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

지표와 2026-11-07 측정값(화면 없이, 한 바퀴 + heartbeat 끊기, 실시간 비율 1.0):

| 지표 | 기준(안내 4절) | 측정 |
|---|---|---|
| 경로 연장 직후 0.5 s 안에 `cmd_vel` 선속도·각속도가 모두 0인 채 0.1 s 넘게 | 0 | 연장 17회 중 0 |
| 끝 0.15 m 안에 들어와 성공까지 | 6 s 넘는 일 0(R13) | 최대 2.47 s |
| 성공 때 실제 위치와 목표 거리 | 0.15 m 안(R4) | 최대 0.045 m |
| heartbeat 마지막 신호부터 정지까지 | 1.5 s 안(R10) | `cmd_vel` 0까지 1.555 s, 실제 정지까지 1.832 s(그동안 0.353 m) |
| `PATH_NOT_CONNECTED` | 정상 주행에서 0 | 0 |
| 보고 위치 오차 ε(AMCL) | 작을수록 좋음(R3, 추정 0.03 m) | 평균 0.029 m, p95 0.050 m, 최대 0.059 m |

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
