# 자유바닥 관제 실험 결과

`traffic_mode: freefloor`로 바꾸면 레인 대신 0.1 m 칸 잠금과 바닥 경로로 핑키를 움직인다. 주문, 로봇 계약, 시나리오는 레인 관제와 같다. 기본값은 `lanes`이고, `lanes`는 실험 전과 똑같이 돈다(기존 시험을 하나도 고치지 않고 모두 통과).

모든 숫자는 머리 없는 가짜 세계(SimWorld/FloorWorld, FakePinky 동작 모델)에서 나왔다. Gazebo와 실물 로봇에서는 돌리지 않았다.

## 돌려 보기

저장소 꼭대기에서(Windows python 또는 WSL python3):

```bash
# 레인 대 자유바닥: 시나리오 3개 x 씨드 20개 x 두 방식 (약 5분)
PYTHONPATH="pc/fleet_core:sim/fake_robots" python3 tools/compare_floor.py --seeds $(seq 1 20) --out docs/freefloor/compare.csv

# 주문 100건 긴 실행
PYTHONPATH="pc/fleet_core:sim/fake_robots" python3 tools/compare_floor.py \
  --scenarios tools/scenarios/long_orders_100.yaml --seeds 1 2 3 --out docs/freefloor/compare_long.csv

# 자유바닥 시험만
cd pc/fleet_core && PYTHONPATH=".:../../sim/fake_robots" python3 -m pytest -q test/test_floor_*.py test/test_traffic_mode.py

# 빈 바닥 그림
python3 tools/floor_view.py --map floor_map --out /tmp/floor.html
```

Windows python에서는 `PYTHONPATH`를 `;`로 잇는다.

### Gazebo로 돌려 보기

Pinky Pro 3대를 Gazebo에 띄우고 진짜 관제(`fleet_manager`)를 자유바닥으로 돌린다. `traffic_mode:=freefloor`가 관제 파라미터를 `traffic_mode: freefloor`로 바꾼 사본(`<result_dir>/fleet_params.yaml`)을 만들어 모든 노드에 준다. RViz(`rviz:=fleet`)에는 쥔 칸이 로봇 색 사각형으로, 물체 앞을 막은 칸이 빨강으로 나온다.

```bash
cd ~/ros2camp/ff_ws
source /opt/ros/jazzy/setup.bash && source ~/pinky_deps_ws/install/local_setup.bash
colcon build --base-paths ~/ros2camp/roscamp-freefloor --packages-up-to fleet_gz_sim fake_robots fleet_manager
source install/local_setup.bash
export ROS_DOMAIN_ID=53 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE FASTDDS_ENVIRONMENT_FILE FASTDDS_BUILTIN_TRANSPORTS
ros2 launch fleet_gz_sim gz.launch.py scenario:=basic_orders traffic_mode:=freefloor sim_dock_stub:=true result_dir:=gz_freefloor
```

- 화면 없이: `gui:=false rviz:=none`. 레인과 비교: `traffic_mode:=lanes`(또는 빼기).
- 창고 지도: 시나리오의 `map: warehouse`가 Gazebo 경기장과 레인 지도를 고르고, 레인 지도의 `floor_map` 키가 자유바닥 지도(`fleet_core/data/floor_map_warehouse.yaml`)를 고른다. 관제·모니터·fleet_check·fleet_view가 같은 레인 지도를 받으므로 같은 바닥 지도를 쓴다.
  `ros2 launch fleet_gz_sim gz.launch.py scenario:=gz_warehouse traffic_mode:=freefloor sim_dock_stub:=true result_dir:=gz_freefloor_warehouse` (장애물까지: `obstacles:=warehouse`)
- 창고 headless 결과(`tools/compare_floor.py --maps warehouse --scenarios warehouse_rush --seeds 1 2 3 4 5`, 8c9320c): lanes 5/5 끝남(마지막 주문 보고 1306.2 s 평균), freefloor 4/5(1372.0 s 평균, 1354.1~1406.5). 끝난 freefloor 4번은 모두 36건을 내보냈고 칸 위반·격리 0, 로봇 사이 최소 거리 0.286~0.291 m. 다만 0.5 m 통로에서 마주치면 DEADLOCK_UNRESOLVED가 런마다 3~14번 나고(새 경로로 풀림), seed 2는 시작 0.1 s에 잡음 낀 보고가 0.35 m 옆 대기 칸 로봇의 칸을 잡아 FloorWorld 불변식에서 멈춘다. 대기 칸 P-1~P-3 간격 0.35 m가 몸 칸(반지름 + 여유 0.13 m, 0.1 m 칸)과 보고 잡음(σ 0.03 m)에 빠듯해서, 주문 6건만 돌리면 세 대가 대기 칸에 들어가다 900 s 안에 못 끝나는 seed도 있다(3, 6, 8).
- 결과는 `result_dir`에 시나리오 판정과 `fleet_check.json`(로봇 사이 최소 거리, 정지 횟수, 쥔 칸 밖 위치 수)으로 남는다. 시나리오가 끝나면 launch가 끝난다.
- 도메인은 53~60, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. launch가 격리를 확인하고 아니면 멈춘다.

## 결과 (52737c1, Windows python, 같은 기계에서 연달아)

| 시나리오 | 방식 | 끝남 | 마지막 주문 보고(s) 평균 (최소~최대) | 정지 횟수 평균 | 기다린 시간 합(s) 평균 | 달린 거리(m) 평균 | 로봇 사이 최소 거리(m) | tick ms 평균 / p99 최대 / 최대 | 비켜 서기 | 미해결 교착 | 격리 | 칸 위반 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| basic_orders | lanes | 20/20 | 177.4 (176.4~178.0) | 8.2 | 103.7 | 56.0 | 0.400 | 0.10 / 0.35 / 3.96 | - | 0 | 0 | - |
| basic_orders | freefloor | 20/20 | 186.3 (184.1~192.3) | 10.6 | 43.5 | 49.6 | 0.346 | 1.12 / 5.98 / 68.30 | 0 | 0 | 0 | 0 |
| order_burst | lanes | 20/20 | 265.3 (264.9~265.8) | 12.2 | 138.2 | 82.8 | 0.400 | 0.10 / 0.27 / 72.47 | - | 0 | 0 | - |
| order_burst | freefloor | 20/20 | 275.1 (272.0~282.1) | 18.1 | 56.9 | 78.5 | 0.336 | 1.19 / 4.95 / 96.85 | 0 | 0 | 0 | 0 |
| blocked_lane | lanes | 20/20 | 195.4 (195.0~195.9) | 10.2 | 137.5 | 56.0 | 0.307 | 0.11 / 0.34 / 1.68 | - | 0 | 0 | - |
| blocked_lane | freefloor | 20/20 | 205.5 (200.2~215.0) | 13.9 | 54.6 | 50.6 | 0.297 | 1.06 / 5.73 / 93.64 | 1 | 0 | 0 | 0 |
| long_orders_100 (씨드 1~3) | lanes | 3/3 | 2167.3 (2166.0~2168.8) | 18.0 | 492.5 | 686.4 | 0.400 | 0.09 / 0.20 / 0.99 | - | 0 | 0 | - |
| long_orders_100 (씨드 1~3) | freefloor | 3/3 | 2300.8 (2292.5~2312.1) | 145.0 | 490.3 | 725.7 | 0.297 | 1.28 / 3.98 / 92.05 | 1 | 0 | 0 | 0 |

읽는 법:

- 시나리오 3개에서 자유바닥은 기다린 시간이 절반 아래로 줄고 길도 짧지만, 일은 5~6% 늦게 끝난다. 100건 긴 실행에서는 기다린 시간이 거의 같고(492.5 → 490.3초) 길은 오히려 길다(686.4 → 725.7 m). 시간을 잡아먹는 곳은 작업대·출고 도크다. 도크 접근선은 한 번에 한 대만 들어가고, 다음 로봇은 도크 둘레 빈 자리에서 기다렸다가 접근선을 끝까지 잠근 뒤에야 들어간다. 레인은 앞 로봇 바로 뒤 구역에서 줄을 서서 더 빨리 들어간다.
- 긴 실행에서 정지 횟수가 크게 늘었다(18 → 145). 굴러가는 잠금이 앞 칸을 못 잡아 잠깐 서는 일이다. 가짜 로봇은 잠근 끝에서 서지만 실제 로봇은 감속으로 넘길 수 있는지 모른다(실측하지 않았다).
- 교착 감지(`DEADLOCK_DETECTED`)는 시나리오마다 수십 번 나오지만 모두 재계획으로 풀렸고 비켜 서기는 전체에서 2번, 미해결은 0번이다.
- 로봇 사이 최소 거리는 0.297 m로 몸 두 개(2r = 0.16 m)보다 넓다.
- 칸 위반 0: 첫 MoveTo부터는 매 단계 실제 몸이 닿는 칸이 모두 그 로봇의 칸이었고, 받은 path도 실제 자리부터 그 로봇의 칸 안이었다. 첫 MoveTo 전은 이 0에 들지 않는다: 출발 자리에 처음 올릴 때 보고 위치가 칸 여유(0.05 m)보다 더 틀어져 몸이 칸 밖으로 삐져나온 로봇을 따로 셌다(`placement_outside`: 60번 실행 중 7대. 그때 그 칸을 남이 쥐지는 않았지만, 그 칸이 그 로봇 것으로 잡혀 있던 것도 아니다). 위치 오차 σ 0.03 m에 여유 0.05 m가 빠듯하다는 뜻이다.
- tick: 평균 1.1~1.3 ms, 실행별 p99는 6 ms 이하. 최대는 20 ms를 넘는 실행이 있다(최대 96.85 ms). 같은 시간에 돈 레인도 최대 72 ms가 나와서 이 기계의 순간 지연(OS·GC)이 섞였을 수 있지만, 원인은 재지 않았다. 따로 잰 실행(order_burst 씨드 3)에서는 2939 tick 중 2개가 20 ms를 넘었다.

## 그림

- `floor_map.html`, `feature_floor_map.html`: 빈 바닥(칸, clear 칸, 고정 선, 도크, 주차).
- `order_burst_seed1.html`: order_burst 씨드 1 자유바닥 재생(0.5초마다 칸 주인, 로봇, 보낸 path).

## 남은 일

- Gazebo·실물 확인. 노드(`fleet_manager`)는 `traffic_mode`를 읽지만 바닥 지도 파일 인자는 없고 기본 `floor_map.yaml`을 쓴다. ROS 판정기는 레인 기준이다.
- 조용히 선 로봇 감시(P4)와 특징 지도 좁은 통로 시나리오(P5)는 하지 않았다.
- 칸 여유와 위치 오차, 감속 정책, 도크 앞 대기 자리 수는 실측으로 정해야 한다.
