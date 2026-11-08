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

- 자유바닥은 기다린 시간이 절반 아래로 줄고 길도 짧지만, 일은 5~6% 늦게 끝난다. 시간을 잡아먹는 곳은 작업대·출고 도크다. 도크 접근선은 한 번에 한 대만 들어가고, 다음 로봇은 도크 둘레 빈 자리에서 기다렸다가 접근선을 끝까지 잠근 뒤에야 들어간다. 레인은 앞 로봇 바로 뒤 구역에서 줄을 서서 더 빨리 들어간다.
- 긴 실행에서 정지 횟수가 크게 늘었다(18 → 145). 굴러가는 잠금이 앞 칸을 못 잡아 잠깐 서는 일이다. 가짜 로봇은 잠근 끝에서 서지만 실제 로봇은 감속으로 넘길 수 있는지 모른다(실측하지 않았다).
- 교착 감지(`DEADLOCK_DETECTED`)는 시나리오마다 수십 번 나오지만 모두 재계획으로 풀렸고 비켜 서기는 전체에서 2번, 미해결은 0번이다.
- 로봇 사이 최소 거리는 0.297 m로 몸 두 개(2r = 0.16 m)보다 넓다.
- 칸 위반 0: 매 단계 실제 몸이 닿는 칸이 모두 그 로봇의 칸이었고, 받은 path도 실제 자리부터 그 로봇의 칸 안이었다. 단, 출발 자리에 처음 올릴 때 보고 위치가 칸 여유(0.05 m)보다 더 틀어져 몸이 칸 밖으로 삐져나온 로봇을 따로 셌다(`placement_outside`: 60번 실행 중 7대, 첫 MoveTo 전, 그 칸을 남이 쥐지는 않았다). 위치 오차 σ 0.03 m에 여유 0.05 m가 빠듯하다는 뜻이다.
- tick: 평균 1.1~1.3 ms, 실행별 p99는 6 ms 이하. 최대는 20 ms를 넘는 실행이 있다(최대 96.85 ms). 같은 시간에 돈 레인도 최대 72 ms가 나와서 이 기계의 순간 지연(OS·GC)이 섞여 있다. 따로 잰 실행(order_burst 씨드 3)에서는 2939 tick 중 2개가 20 ms를 넘었다.

## 그림

- `floor_map.html`, `feature_floor_map.html`: 빈 바닥(칸, clear 칸, 고정 선, 도크, 주차).
- `order_burst_seed1.html`: order_burst 씨드 1 자유바닥 재생(0.5초마다 칸 주인, 로봇, 보낸 path).

## 남은 일

- Gazebo·실물 확인. 노드(`fleet_manager`)는 `traffic_mode`를 읽지만 바닥 지도 파일 인자는 없고 기본 `floor_map.yaml`을 쓴다. ROS 판정기는 레인 기준이다.
- 조용히 선 로봇 감시(P4)와 특징 지도 좁은 통로 시나리오(P5)는 하지 않았다.
- 칸 여유와 위치 오차, 감속 정책, 도크 앞 대기 자리 수는 실측으로 정해야 한다.
