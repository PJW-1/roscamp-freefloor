# 자유바닥 관제 실험 기록 (2026-11-09)

`자유바닥_관제_실험_지시서_2026-11-08.md`를 따른다. 사용자 지시가 지시서와 다른 곳은 사용자 지시를 따랐다.

- 저장소 `PJW-1/roscamp-freefloor`(비공개). 시작점은 지시서의 `v1.1`이 아니라, 사용자 지시대로 개인 저장소 `fms-work-order` db2ab58(`v1.1` + 핑키 자가 복구)이다. `v1.1`(04e0908)은 그 조상이다. `main`은 db2ab58로 push했고 태그 `v1.1`도 push했다. 작업 브랜치는 `freefloor`다.
- 오늘 밤은 P0 → P1 → P2 → P3 → P6 → P7 순서로 했다. P4와 P5는 하지 않았다(사용자 지시: 선택).
- Gazebo는 돌리지 않았다(사용자 지시). 확인은 FakePinky 시험과 머리 없는 시나리오 실행(SimWorld)으로만 했다. 실물 로봇에는 접속하지 않았다.
- ROS 시험 도메인은 60~69만 썼다(`fleet_manager` 시험은 공용 잠금 `~/.fleet_manager_tests.lock`을 잡고 돌렸다).
- 실행하지 않은 것은 실행하지 않았다고 적는다.

## 기준선(db2ab58)

- `fleet_core`: 1051 passed(WSL, 순수 pytest, 2분 29초).
- `fake_robots`: 425 passed(4분 32초).
- `fleet_manager`: 첫 실행은 2 failed, 82 passed, 12 errors(11분 50초). 실패와 오류 이름은 출력 끝 12줄만 남겨서 다 보지 못했다. 보인 오류는 모두 `test_node_wiring.py`의 `RuntimeError`였다. 같은 시각에 다른 작업이 ROS 시험을 돌리고 있었다. 이 실험의 첫 `fleet_manager` 변경(971353b) 뒤 16e8f81에서 다시 돌렸을 때는 96 passed(12분 37초)였다. 기준선 실패를 원인까지 따로 확인하지는 않았다.

## P0. `traffic_mode` 스위치

| 곳 | 바꾼 것 |
|---|---|
| `common/params.py`, `fleet_params.yaml` | `traffic_mode: lanes`(기본). 값이 없던 기록은 `lanes`로 재생된다 |
| `traffic/modes.py`(새) | `build_traffic`: `lanes`면 지금의 `TrafficController` 그대로, `freefloor`면 `FloorController`. 모르는 값은 `ValueError` |
| `manager.py` | 교통 층을 `build_traffic`로 만든다. `FleetManager(..., floor=None)` 인자 하나를 더했다(바닥 지도). `check_recordable`: freefloor 기록은 아직 재생할 수 없어 거부 |
| `fleet_manager_node.py` | 기록 파일을 열기 전에 `check_recordable`을 부른다(Codex 지적) |
| `traffic/floor/controller.py`(새) | 이 파트에서는 뼈대만(레인 동작을 물려받음) |

- 빨강: `test_traffic_mode.py`를 먼저 쓰고 돌렸다. `ModuleNotFoundError: fleet_core.traffic.floor`로 수집 오류 1건.
- 초록: 7 passed. `fleet_core` 전체 1058 passed.
- `lanes`의 기존 시험은 하나도 고치지 않았다.

Codex 검수(db2ab58..7446532), 지적 1건:

- [확정, 고침 971353b] 노드가 `--record` 파일을 `"w"`로 먼저 열고 그다음 `record_to`가 freefloor를 거부해서, 옛 기록 파일이 지워질 수 있었다. 확인: `fleet_manager_node.py` 66~69행이 그 순서였다. 고침: `FleetManager.check_recordable()`을 만들고 노드가 파일을 열기 전에 부른다. 빨강 1 failed, 7 passed → 초록 36 passed(`test_traffic_mode.py`, `test_record_replay.py`). 노드 쪽 변경은 `fleet_manager` 96 passed로 확인했다(노드 시험이 이 경로를 직접 부르지는 않는다).

### 공부 노트 (P0)

1. 핵심 개념
   - 스왑: 주문 코어와 `FleetManager` 바깥 API는 그대로 두고 교통 층만 바꿔 끼운다. 그래서 같은 주문·같은 시나리오로 두 방식을 비교할 수 있다.
   - 기본값이 있는 새 파라미터: 데이터클래스 필드에 기본값을 주면 그 값이 없던 옛 기록도 그대로 읽힌다.
   - 지연 import: 실험 모듈은 고를 때만 읽어서 `lanes`는 실험 코드 없이도 같게 돈다.
2. 바뀐 파일·함수
   - `traffic/modes.py` `build_traffic`: 모드에 맞는 교통 제어기를 만든다.
   - `manager.py` `FleetManager.__init__`, `check_recordable`: 교통 층 선택, 기록 가능 여부.
3. 핵심 코드

   ```python
   def build_traffic(site, params, robots, clock, next_goal_id, floor=None):
       mode = params.traffic_mode                     # 파라미터로 고른다
       if mode == LANES:
           return TrafficController(site, params, robots, clock, next_goal_id)  # 지금 코드 그대로
       if mode == FREEFLOOR:
           from fleet_core.traffic.floor.controller import FloorController  # 고를 때만 읽는다
           return FloorController(site, params, robots, clock, next_goal_id, floor)
       raise ValueError(...)                          # 모르는 값은 시작부터 막는다
   ```

4. 면접 한 문장: "교통 층을 파라미터 하나로 갈아 끼울 수 있게 해서, 주문·로봇 계약·시나리오를 하나도 바꾸지 않고 레인 관제와 자유바닥 관제를 같은 숫자로 비교했습니다."

## P1. 바닥 지도와 칸, 칸 잠금, 그림 도구

| 곳 | 바꾼 것 |
|---|---|
| `data/floor_map.yaml`(새) | Gazebo `lane_arena.pgm`(복사본, 시험이 바이트 단위로 같은지 본다)을 읽는다. `cell_m` 0.10, `robot_radius_m` 0.08(트레이 미포함, 레인과 같은 값), `cell_margin_m` 0.05, 도크·주차 위치(레인 지도와 같다), 고정 선 4개(도크 접근 `work_in` Q→W, `outbound_in` QO→O, 도크에서 나감 `work_out` W→A, `outbound_out` O→B) |
| `data/feature_floor_map.yaml`(새) | 특징 지도 좌표의 사각형 바닥. 좁은 통로 TUN은 같은 칸을 쓰는 고정 선 둘(`tunnel_east`, `tunnel_west`) |
| `traffic/floor/floor_map.py`(새) | PGM 읽기, 칸, `free`(칸 전체가 빈 바닥), `clear`(칸 사각형 어디서든 벽까지 반지름 + 여유 이상) |
| `traffic/floor/cells.py`(새) | `disk_cells`, `swept_cells`, `segment_cells`(격자 걷기), `CellLocks` |
| `tools/floor_view.py`(새) | 칸 주인·로봇·경로 기록을 재생하는 html 한 장(바깥 스크립트 없음). `docs/freefloor/floor_map.html`, `feature_floor_map.html`은 빈 바닥 그림 |

- 지도 숫자: lane_arena는 40 × 24칸, free 796칸, clear는 처음(칸 중심 기준) 593칸이었고 Codex 지적으로 칸 전체 기준으로 바꾼 뒤 더 줄었다. 특징 바닥은 27 × 18칸.
- 이 경기장에는 양옆이 막힌 좁은 통로가 없다. 그래서 고정 선은 도킹 접근과 도크에서 나가는 선뿐이다.
- 빨강: 시험 3개 파일을 먼저 쓰고, 그 전에 써 둔 `floor_map.py`를 잠시 치운 채 돌렸다. 수집 오류 3건(`floor_view`, `cells`, `floor_map` 없음). `floor_map.py`는 시험보다 먼저 썼다는 점을 밝혀 둔다.
- 초록: 28 passed. `fleet_core` 전체 1087 passed.

Codex 검수(971353b..2e822b7), 지적 8건. 모두 코드로 확인했다:

1. [확정, 고침] `clear`를 칸 중심에서만 재서, 칸 가장자리의 로봇 중심은 벽에 반지름 + 여유보다 가까울 수 있었다. 계획기는 점이 어느 칸에 드는지만 보므로 안전 문제다. 칸 사각형에서 벽까지로 바꿨다. 빨강 1 failed, 11 passed(칸 안 9점 시험).
2. [확정, 고치지 않음] 실패한 요청은 그때 남이 쥔 칸만 대기로 남긴다. 제어기가 매 tick 다시 시도해서 대기 기록이 매 tick 새로 쓰이므로 그대로 두었다. 순서(FIFO)도 없다. 칸 잠금은 공정한 줄 서기 대신 재계획과 비켜 서기로 푸는 설계다.
3. [확정, 고침] 로봇 이름에 `</script>`가 있으면 html이 깨졌다. JSON의 `<`를 `<`로 쓰고 범례를 `textContent`로 만든다.
4. [확정, 고침] 칸이 픽셀의 정수배가 아니면 벽 픽셀을 놓쳤다. 정수배가 아니면 `FloorMapError`.
5. [확정, 고침] 생성기를 넘기면 검사에서 다 써 버려 빈 잠금으로 성공했다. 입력을 먼저 집합으로 만든다.
6. [확정, 고침] PGM `maxval`을 무시했고, 잘린 헤더에서 무한 반복했다. 255만 읽고 잘린 헤더는 오류로 낸다(잘린 헤더 시험은 고치기 전 20초 제한에 걸려 결과 없이 끝났다).
7. [확정, 고침] 사각형 장애물을 픽셀 중심 포함으로만 칠해서 얇은 상자가 사라졌다. 조금이라도 겹치는 픽셀을 막는다.
8. [확정, 고침] 원이 칸 경계에 딱 닿으면 그 칸이 빠졌다. 후보 범위를 한 칸 넓혔다.

- 빨강: 6 failed, 29 passed, 1 deselected, 그리고 잘린 헤더 시험은 20초 제한에 걸림. 초록: 46 passed(지도·칸·그림·계획 시험). 커밋 6c50a0c.

### 공부 노트 (P1)

1. 핵심 개념
   - 칸 잠금: 바닥을 0.1 m 칸으로 나누고 칸마다 주인을 하나만 둔다. 레인 구역보다 훨씬 잘게 잠근다.
   - 몸이 쓰는 칸: 로봇을 반지름 + 여유인 원으로 보고, 그 원이 닿는 칸을 모두 잡는다. 경로를 따라 쓸고 지나가는 칸은 경로 위 촘촘한 점마다 원을 놓은 합이다.
   - 한꺼번에(all-or-nothing): 늘릴 칸이 하나라도 남의 것이면 하나도 잡지 않는다. 일부만 잡고 서로 나머지를 기다리는 꼴을 막는다.
   - clear 칸: 로봇 중심이 칸 안 어디에 있어도 몸이 벽에서 여유만큼 떨어지는 칸. 계획기는 이 칸만 지난다.
2. 바뀐 파일·함수: `floor_map.py` `FloorMap`(읽기, `cell_of`, `box_distance`, `_box_gap_to_wall`), `cells.py` `disk_cells`, `swept_cells`, `segment_cells`, `CellLocks.acquire_all`, `tools/floor_view.py` `render`.
3. 핵심 코드

   ```python
   def acquire_all(self, robot, cells):
       cells = set(cells)                                    # 생성기도 한 번에 굳힌다
       taken = sorted(c for c in cells if self._holder.get(c) not in (None, robot))
       if taken:                                             # 하나라도 남의 것이면
           self._waiting[robot] = tuple(taken)               # 누구를 기다리는지만 적고
           return False                                      # 하나도 안 잡는다
       ...                                                   # 모두 잡는다
   ```

   ```python
   def swept_cells(floor, points, radius):
       step = floor.cell_m / 4                               # 점 간격: 칸의 1/4
       reach = radius + step / 2                             # 점 사이 구간도 덮도록 반 간격 더
       for x, y in sample_line(points, step):
           cells |= disk_cells(floor, x, y, reach)           # 점마다 원이 닿는 칸
   ```

4. 면접 한 문장: "칸을 촘촘하게 잠가도 늘릴 칸을 한꺼번에만 잡게 해서, 두 로봇이 서로 반쪽씩 쥐고 기다리는 상태가 생기지 않게 했습니다. 그래도 남는 대기 순환은 재계획과 비켜 서기로 풉니다."

## P2. 한 대: 계획 → 굴러가는 잠금 → MoveTo 교체 → 도착·도킹

| 곳 | 바꾼 것 |
|---|---|
| `traffic/floor/planner.py`(새) | 칸 위 8방향 A*(모서리 자르기 금지) → 시선 지름길(Theta* 방식, 지나는 칸을 격자 걷기로 정확히 본다) → 꺾임을 반지름 0.15 m 이하 원호로. 도크로 가는 다리는 고정 접근선으로 끝나고, 도크에서 나가는 다리는 나가는 선부터 따른다. 자유 경로가 고정 선 칸을 지나면 거리 4배 |
| `traffic/floor/controller.py` | `FloorController`: 구역 대신 `CellLocks`. 경로를 따라 몸이 쓸 칸을 `floor_lock_ahead_m`(0.5 m) 앞까지 한꺼번에 잡고, 안 되면 `floor_extend_step_m`(0.1 m)씩 짧게 다시 한꺼번에. 고정 선은 끝까지 통째로. 보내는 path는 잠근 끝까지만, 잠금이 늘면 MoveTo 교체. 뒤 칸은 보고 3번이 0.2 m 밖일 때 놓는다. 단계·도킹·배차·선택은 물려받는다 |
| `params` | `floor_lock_ahead_m` 0.5, `floor_extend_step_m` 0.1, `floor_replan_s` 1.0, `floor_plan_window_s` 8.0(모두 제안값) |
| `test/floor_world.py`(새) | SimWorld의 자유바닥판. 레인 검사 대신 매 단계: 실제 몸(r)이 닿는 칸 ⊆ 쥔 칸, 받은 path(실제 자리부터)의 몸 칸 ⊆ 쥔 칸, 로봇 사이 ≥ 2r, 물건 수 |

- 빨강(계획기): `planner.py`를 먼저 썼고 시험을 쓴 뒤 치운 채 돌렸다(수집 오류 1건). 되돌리고 돌리니 2 failed, 8 passed: 하나는 시험 지도가 칸 전체 기준 clear에 너무 좁았고(시험 지도를 넓힘), 하나는 실제 결함이었다(원호를 점으로만 검사해서 피해야 할 칸 모서리를 스침 → 격자 걷기로 바꿈). 초록 10 passed.
- 빨강(한 대 주행): `test_floor_single.py`를 뼈대 제어기(레인 동작)에 돌렸다. 4 failed, 6 errors(`ZoneLocks`에 `held_set` 없음 등, 칸 잠금이 없어서). 초록 10 passed. `fleet_core` 전체 1115 passed. `fleet_manager` 96 passed(16e8f81).
- 한 대 결과(씨드 1, 주문 1건): 출고까지 끝나고 대기 칸에 섰다. 도중 정지 0번(baseline의 `stops` 정의). 시험은 씨드 1~5에서 같은 것을 본다.

Codex 검수(6c50a0c..16e8f81), 지적 10건:

1. [확정, 고침] 보낸 path의 첫 구간(보고 자리 → 경로 위 점)은 잠그지 않았다. 이제 보낼 꺾은선 그대로 쓸어서 한꺼번에 잡은 뒤에만 보낸다.
2. [확정, 고침] 새 MoveTo를 보내면 옛 MoveTo의 남은 칸 보호가 바로 사라졌다(새 goal이 거절되면 더). 이제 goal마다 남은 경로를 두고 그 goal의 결과(거절·미전송·분실 포함)가 오거나 로봇이 goal 없이 서 있을 때만 지운다.
3. [확정, 고침] 통신 복구가 레인 규칙으로 칸을 놓고 지점 칸 하나만 다시 잡았다. 자유바닥판 `_finish_recovery`: 지금 몸 칸을 한꺼번에 잡고(못 잡으면 `RECOVERY_CONFLICT` 격리) 그 자리에서 다시 계획한다.
4. [확정, 고침] 주행 중 보고 자리의 몸 칸을 다시 잡거나 검사하지 않았다. 이제 보고마다 몸 칸을 쥐었는지 보고, 아니면 한꺼번에 잡고, 못 잡는 보고가 3번 이어지면 goal을 거두고 `FLOOR_OFF_CELLS`.
5. [확정, 고치지 않음, 영향 작음] 목표 도크 0.15 m 안에서 다리를 시작하면 접근선을 잠그지 않고 도착한다. 그때 로봇은 이미 도크 위라 몸 칸을 쥐고 있다.
6. [확정, 고침] 원호가 원래 경로에 없던 고정 선 칸을 지날 수 있었다. 원호에도 고정 선 진입 검사를 한다.
7. [추측, 틀림] "`_committed`가 출발부터 참" — `m.path`의 마지막이 W라 commit 칸이 남은 경로에 있어 거짓이다. 접근선을 잠그면(W 칸을 쥐면) 참이 된다. 고치지 않았다.
8. [확정, P3에서] 막힘·조용히 선 로봇 감시가 돌지 않았다. 막힘은 P3에서 만들었다. 조용히 선 로봇 감시(`_watch_idle`)는 P4 몫이라 하지 않았다.
9. [추측, 고치지 않음] 주차 자리를 중심 칸 하나로만 본다. 다른 로봇이 그 자리 몸 칸 일부를 쥐면 고른 자리에서 기다릴 수 있다.
10. [확정, 고침] FloorWorld가 path 첫 구간을 검사하지 않았다. 이제 실제 자리부터 검사한다.

- 빨강 4 failed, 11 passed → 초록 25 passed(158502e). `fleet_core` 전체 1120 passed. 통신 두절 시나리오 시험은 고치기 전에도 통과했다(빨강이 아니었다).

짧은 재검수(16e8f81..158502e), 지적 6건:

1. [확정, 고침] 두절 동안 마지막 goal이 성공하면 `leg.done`이 남아 복구 뒤 엉뚱한 자리에서 도착 처리할 수 있었다. 다시 계획하면 `done`을 지운다.
2. [확정, 고침] 복구 뒤 이미 벗어난 도크의 나가는 선을 다시 따랐다. 나가는 선은 로봇이 그 선 시작점 0.15 m 안에 있을 때만 쓴다.
3. [확정, 고침] `DOCKING`을 위치와 상관없이 유지했다. 레인판처럼 도크 위일 때만 유지하고, 아니면 배차를 다시 내거나 IDLE로 둔다.
4. [확정, 고침] 남의 칸 위 보고가 정확히 3번째일 때만 멈췄다. 3번 이상이면 멈춘다. 이 고침을 직접 드러내는 시험은 만들지 못했다(새 시험은 고치기 전에도 3번째에 멈춘다).
5. [추측, 고치지 않음] 옛 goal의 남은 경로를 지금 자리에서 가장 가까운 점부터 지키므로 일찍 놓을 수 있다. 새 goal이 시작되면 로봇은 옛 경로를 따르지 않으므로 그대로 둔다.
6. [확정] 새 시험이 약하다. 두 로봇 시험 하나를 더했다.

- 빨강 3 failed, 15 passed. 그중 하나는 시험이 출력을 잘못된 곳(제어기)에서 읽어서 실패한 것이었다(관리자가 바로 넘겨 가져감). 시험을 고친 뒤 18 passed(da1328b). `fleet_core` 전체 1123 passed.

### 공부 노트 (P2)

1. 핵심 개념
   - Any-angle 경로: 칸 위 A*의 계단 모양을 "보이는 가장 먼 점으로 곧게"로 펴고, 꺾인 곳은 작은 원호로 둥글린다. Nav2 FollowPath가 받는 모양이 된다.
   - 굴러가는 잠금: 로봇 앞 0.5 m만 미리 잠그고 매 tick 앞으로 늘린다. path는 잠근 끝까지만 보내고 늘면 바꿔 보낸다. 로봇은 서지 않고 계속 간다.
   - "안전은 잠금, 효율은 계획": 계획이 틀려도 로봇은 쥔 칸 밖으로 가는 path를 받지 않는다.
   - 고정 선: 도킹 접근처럼 정해진 선은 들어가기 전에 끝까지 한꺼번에 잡는다.
2. 바뀐 파일·함수: `planner.py` `FloorPlanner.plan/leg/astar/shortcut/round_corners`, `Route`; `controller.py` `_start_leg`, `_plan`, `_drive`, `_candidates`, `_extend`, `_send`, `_release_cells`, `_watch_lane_error`, `_finish_recovery`, `on_move_result`; `floor_world.py` `FloorWorld`.
3. 핵심 코드

   ```python
   def _candidates(self, leg):
       want = min(route.length, leg.s + lock_ahead)          # 0.5 m 앞까지 바란다
       ends = [want, want - 0.1, ...]                        # 안 되면 0.1 m씩 짧게
       for e in ends:
           if e가 고정 선 (s0, s1) 안이면: picks = [s1, s0]   # 선 끝까지 통째로, 아니면 선 앞에서 멈춤
   ```

   ```python
   def _extend(self, m, leg):
       for end in self._candidates(leg):
           cells = 경로[s..end]가 쓸 칸 | 몸 칸 | 보낼 path가 쓸 칸
           if self.zones.acquire_all(m.name, cells):         # 한꺼번에만
               leg.s_lock = end                              # 여기까지 가도 된다
               return True
   ```

   ```python
   keep = 몸 칸 | 잠근 앞 칸 | 아직 끝나지 않은 MoveTo들의 남은 칸
   for cell in 쥔 칸:
       if cell not in keep and 연속 보고 3번이 0.2 m 밖:    # 몸이 확실히 지난 뒤에만
           self.zones.release(m.name, cell)
   ```

4. 면접 한 문장: "로봇에게는 잠근 칸 끝까지만 경로를 주고 잠금이 늘 때마다 경로를 바꿔 보내서, 로봇은 서지 않고 달리지만 쥐지 않은 칸으로 가라는 명령은 한 번도 받지 않습니다."

## P3. 여러 대: 예약 벌점, 재계획, 대기 순환 → 비켜 서기, 막힘 표시, 도크 접근선 줄

| 곳 | 바꾼 것 |
|---|---|
| `controller.py` `_route`, `_inflate` | 남이 쥔 칸을 피해 계획한다. 계획기는 중심이 지나는 칸만 보므로 남의 칸을 몸 반지름만큼 넓혀서(`_reach`: 칸 사각형 사이 거리가 몸 반지름보다 가까운 칸) 넘긴다. 길이 없으면 피하지 않고 다시 찾는다(잠금이 안전을 지킨다) |
| `_reservations`, `_penalty_for` | 남의 경로를 계약 속도(`detour_speed_mps` 0.2)로 `floor_plan_window_s`(8초) 앞까지 달릴 때 칸마다 머무는 시간. 그 시간에 닿는 칸은 거리 0.5 m 벌점(시간 벌점 A*, 협조 A*의 가벼운 꼴) |
| `_replan_if_stuck` | 잠금이 `floor_replan_s`(1초) 동안 늘지 않고 앞이 짧으면 남이 쥔 칸을 아예 빼고 다시 계획한다. 고정 선 안에서는 하지 않는다 |
| `_check_deadlock`, `_sidestep` | 대기 그래프 순환이 `deadlock_grace_s` 이어지면 우선순위가 낮은 로봇부터 남의 칸과 남이 1.2 m 안에 가려는 칸에서 벗어난 가장 가까운 빈 자리로 비켜 선다(고정 선 위·도킹 중이면 다음 로봇). 3초 뒤 원래 다리를 다시 계획한다. 비켜 설 곳이 없거나 비켜 선 뒤에도 순환이 남으면 `DEADLOCK_UNRESOLVED`, 그 뒤 `deadlock_unresolved_s`면 격리 |
| `_note_block_report`, `_mark_blocked`, `_take_ahead` | BLOCKED가 `blocked_mark_s` 이어지면 goal을 거두고 앞 0.3 m 칸을 막힘(주인 `#blocked`)으로 표시한다. 로봇이 쥐고 있던 앞 칸은 거둔 goal이 끝난 뒤에 표시로 옮긴다. BLOCKED가 아닌 보고가 3번 오면 지우고 다시 계획(`FLOOR_BLOCKED`, `FLOOR_CLEARED`) |
| `_admitted`, `_plan_wait` | 도크 접근선은 한 번에 한 대. 다른 로봇이 그 선으로 가는 중이거나 선 칸을 쥐고 있으면, 접근선 시작에서 0.3~1.2 m, 어떤 고정 선 칸에도 닿지 않는 빈 자리에서 기다린다(서 있는 동안 쥐는 반경 `release_distance_m`까지 본다). 선이 비면 먼저 기다린 로봇부터 |
| `planner.py` `_rest_of_line` | 접근선에서 0.12 m 안에 선 로봇은 선 첫 점으로 되돌아가지 않고 그 자리에서 선을 잇는다(그 한 걸음이 clear 칸만 지날 때) |
| `cells.py` | `held_by_others` |
| `tools/compare_floor.py`(새) | 같은 시나리오·씨드를 두 방식으로 돌려 baseline 열 + `min_gap_m`, `path_m`, `tick_ms_mean/max`, `slowdowns`, `replans`, `sidesteps`, `deadlocks`, `unresolved`, `isolations`, `violations` |

### 고친 순서와 시험

- 빨강: `test_floor_multi.py`를 P2 제어기에 돌렸다. 시나리오 5개 모두 끝나지 않음(basic_orders 씨드 1·2·3은 900초 제한에서 출고 2·1·1건, order_burst는 1200초에 1건, blocked_lane은 900초에 4건), `FLOOR_BLOCKED` 없음, `_reservations`·`_penalty_for` 없음(`AttributeError`). 실패 8건(실패 줄 `E`를 셌다. 출력을 앞 20줄만 남겨 pytest 요약 줄은 보지 못했다).
- 첫 구현(예약·재계획·비켜 서기·막힘) 뒤 basic_orders 씨드 1·2·3은 233.8 / 224.1 / 204.5초에 끝났지만 order_burst 씨드 1은 1200초에도 세 대가 모두 `TO_WORK`였다. 찍어 보니:
  1. 세 대가 한꺼번에 작업대로 가서 접근선 시작(Q) 둘레에 모였다. 한 대는 자유 경로로 접근선 위에 올라서서 Q로 되돌아가야 했고, Q에는 다음 로봇이 서 있었다.
  2. 2-순환과 3-순환이 tick마다 바뀌어서 매번 새 순환으로 쳐서 유예 시간이 차지 않았다.
- 고침 1, 같은 무리의 순환 합치기: 1초 안에 겹치는 순환은 한 일화로 본다. 시험 `test_overlapping_cycles_that_take_turns_are_one_episode`, 빨강(비켜 서기 없음) → 초록.
- 고침 2, 접근선 위에서 잇기: `test_a_robot_already_on_the_in_line_joins_it_where_it_stands`, 빨강 1 failed, 10 passed → 초록 11 passed. 처음 0.05 m로는 order_burst가 그대로 멈췄다(로봇이 선에서 0.09 m 떨어져 있었다). 0.12 m로 넓히자 539.1초에 끝났지만 isolation 1번, 그리고 basic_orders 씨드 3이 900초에 끝나지 않게 됐다(병합 집합이 커져 엉뚱한 로봇을 격리).
- 고침 3, 도크 접근선 줄: `test_a_second_robot_for_the_same_dock_waits_off_the_lines`, 빨강(`_Leg`에 `waiting` 없음) → 초록. 대기 자리를 몸 칸으로만 보면 서 있는 로봇이 놓지 않는 0.2 m 안 칸이 나가는 선을 막았다 → 서 있는 반경으로 바꿨다.
- 고침 4, 길이 없을 때 옛 경로를 지운다: 줄을 기다리던 로봇이 차례가 됐는데 길이 없으면 옛 대기 경로 끝에서 도착 처리해 도크 밖에서 `DockCmd`를 냈다(FloorWorld 불변식이 잡음: basic_orders 씨드 1·2). `test_a_waiting_robot_with_no_way_to_the_line_does_not_dock_at_its_wait_spot`, 빨강 1 failed(경로가 남음) → 초록. 이 단위 시험에서는 단계가 `TO_WORK`로 남아 단계 검사는 빨강이 아니었다.
- 고침 5, 몸 반지름만큼 넓혀 피하기: 막힌 로봇 하나가 남의 서 있는 칸 옆 경로를 끝없이 다시 받았다(중심 칸은 피했지만 몸 칸이 남의 칸에 닿음). `test_a_route_keeps_the_whole_body_off_cells_others_hold`, 처음 두 배치는 고치기 전에도 통과해서 버리고, 실패가 드러나는 배치(남이 (1.8, 0.95), 목표 (1.0, 0.75))로 빨강 → 초록. 이 고침 뒤 다섯 시나리오가 모두 200~300초에 끝났고 비켜 서기가 0번으로 줄었다.
- 자유바닥 시험 파일 전체: 86 passed(Windows python, 1분 3초). 커밋 7243352.
- `fleet_core` 전체(WSL, 7243352): **1 failed, 1136 passed**. 실패는 `test_fms_boundary.py::test_traffic_has_no_order_or_basket_field`: 교통 코드는 주문 쪽 이름을 쓰면 안 되는데 지역 변수 이름 `order`(비켜 설 차례)가 걸렸다. `turns`로 바꿨다(632ee27).
- 이름을 고친 뒤(그리고 아래 검수 고침을 넣은 뒤) `fleet_core` 전체(WSL, 632ee27): **1143 passed**(3분 36초).
- 다섯 시나리오(Windows python, FloorWorld, 씨드 1): basic_orders 1·2·3은 203.5 / 212.2 / 202.7초, order_burst 291.2초, blocked_lane 219.8초. 모두 끝났고 세 대 모두 PARKED. 비켜 서기 0번, 재계획 1~6번. 칸 불변식 위반 0.
- 100건 긴 실행(새 시나리오 `tools/scenarios/long_orders_100.yaml`, 7243352): 레인 2168.8초, 자유바닥 2292.5초. 둘 다 100건 출고, 위반·격리·미해결 0. 자세한 비교는 P6.

Codex 검수(5c2c788..7243352), 지적 6건:

1. [확정, 고침 632ee27] BLOCKED로 goal을 거둔 바로 뒤, 로봇이 쥔 앞 칸을 놓고 막힘 표시로 넘겼다. 취소 결과가 오기 전에는 로봇이 옛 경로를 계속 따를 수 있으므로 위험하다. 이제 그 칸은 로봇 것으로 두고(`_ahead`), goal이 끝난 뒤 `_take_ahead`가 표시로 옮긴다. 빨강 → 초록.
2. [확정, 고침] 격리·제거된 로봇의 다리가 접근선 줄을 계속 잡았다(`_fail`이 다리를 지우지 않음). 줄 판단은 격리되지 않은 로봇의 다리만 본다(`_in_service`). 빨강 → 초록.
3. [확정, 고침] 합친 순환 집합에서 비켜 설 로봇·격리할 로봇을 골라, 지금 순환에 없는 로봇을 고를 수 있었다. 이제 일화 시간만 합치고 고르는 것은 지금 순환에서만. 시험은 처음엔 고치기 전에도 통과해서(합친 집합을 넘겨받으니 그 안에 있음), "지금 그래프의 순환과 같아야 한다"로 고친 뒤 빨강 → 초록.
4. [확정, 고침] 다른 로봇의 비켜 서기 다리가 남아 있으면 `DEADLOCK_UNRESOLVED`가 끝없이 미뤄졌다. 지금 순환의 로봇만 본다. 빨강 → 초록.
5. [확정, 일부 고침] (가) A*가 벌점에 넘기는 시각이 거리 대신 누적 비용(앞선 벌점·고정 선 배수 포함)이었다. 거리를 따로 센다. 빨강(벌점 100에서 시각 102 m 상당) → 초록. (나) 시선 지름길이 예약 벌점을 보지 않아, 벌점으로 돌아간 길을 다시 곧게 펼 수 있다. 고치지 않았다: 효율 문제이고 안전은 칸 잠금이 지킨다. (다) `>= 0.0` 검사는 늘 참이라 지웠다.
6. [추측 → 확인, 고침] 접근선에 잇는 한 걸음이 clear를 검사하지 않았다. 벽 가까운 칸에서 잇는 시험이 빨강이었다 → 그 걸음이 clear 칸만 지날 때만 잇는다.

- 고친 뒤 자유바닥 시험 104 passed, `fleet_core` 전체 1143 passed(632ee27).

짧은 재검수(7243352..632ee27), 지적 4건:

1. [확정, 고침 8006456] `_follow`가 BLOCKED를 "서 있음"으로 보고 옛 goal 경로 보호(`_active`)를 지웠다. 그래서 `_take_ahead`가 취소 결과 전에 칸을 옮길 수 있었다. BLOCKED는 goal을 멈춘 것일 뿐이라 결과가 올 때까지 지키게 했다. 빨강(같은 시험에 `_follow`를 끼워 넣음) → 초록.
2. [확정, 고침] 통신이 끊긴 로봇은 `available`이 거짓이라 줄을 내놓았고, 복구 전후로 두 대가 같은 선에 들어가도 되는 상태가 됐다(칸 잠금이 충돌은 막는다). 통신 두절은 차례를 지키고, 격리·제거만 내놓는다(`live`). 빨강 → 초록.
3. [확정, 고침] 미해결로 찾은 로봇과 격리하는 로봇이 다를 수 있었다(그 사이 순환이 줄어듦). 찾은 로봇을 기억하고 그 로봇이 아직 순환에 있을 때만 격리한다. 없으면 지금 순환을 처음부터 다시 판단한다. 빨강(`('b', ['a'])`) → 초록.
4. [추측, 고치지 않음] 시간 벌점이 있는 A*는 같은 칸에 다른 시각에 닿는 경로를 하나로 합쳐서 최적이 아닐 수 있다. 맞는 지적이지만 벌점은 효율용이고, 시각까지 상태로 두면 탐색이 커진다.

- 고친 뒤 자유바닥 시험 106 passed, `fleet_core` 전체 **1145 passed**(WSL, 8006456, 3분 37초). 세 시나리오(basic_orders 1, order_burst 1, blocked_lane 1)는 전과 같은 시각에 끝났다(203.5 / 291.2 / 219.8초).

두 번째 짧은 재검수(632ee27..8006456), 지적 3건. 모두 고치지 않았다:

1. [확정, 정책대로] 통신이 끊긴 로봇은 돌아오지 않아도 접근선 차례를 계속 쥔다. 레인판도 통신 두절 로봇의 구역을 격리·운영자 처리 때까지 쥐고 있다. 같은 규칙으로 두었다(통신 두절 로봇이 언제 돌아와 어디로 갈지 모른다).
2. [확정, 정책대로] 취소 결과가 끝내 오지 않고 BLOCKED가 이어지면 옛 경로 칸을 쥔 채 `blocked_timeout_s` 뒤 격리된다. 격리된 로봇은 원래 `confirm_clear`까지 모든 칸을 쥔다(레인판과 같다). 결과 분실이 감지되면 `on_goal_lost`가 푼다.
3. [추측, 고치지 않음] 순환이 만료 때마다 바뀌어 찾은 로봇이 매번 빠지면 격리가 계속 미뤄질 수 있다. 실행에서 본 적은 없다. 남은 위험으로 적는다.

### 공부 노트 (P3)

1. 핵심 개념
   - 안전과 효율을 나눈다: 겹치지 않음은 칸 잠금(한꺼번에 잡기)이 지키고, 계획(벌점, 피하기, 재계획, 비켜 서기, 줄)은 덜 기다리게 할 뿐이다. 계획이 틀려도 부딪히지 않는다.
   - 형상 공간(configuration space): 로봇을 점으로 계획하려면 장애물을 로봇 반지름만큼 넓혀야 한다. 남이 쥔 칸도 장애물이라 같은 방법으로 넓혔다(`_inflate`). 이것을 빼먹은 것이 마지막까지 남은 교착의 원인이었다.
   - 예약 표(reservation table): 남의 경로를 시간 축에 펼쳐서 "그 칸에 그 시각에 누가 있나"를 본다. 협조 A*는 이것을 금지로 쓰고, 여기서는 벌점으로 쓴다.
   - 대기 그래프와 교착: 기다리는 로봇 → 쥔 로봇 화살표에 순환이 생기면 교착이다. 아무도 자기 칸을 놓지 않으니 누군가 물러나야 한다.
   - 줄 세우기(admission control): 좁은 곳(도크 접근선)은 들어갈 수 있는 수를 1로 묶고, 나머지는 그 밖 빈 자리에서 기다리게 한다. 칸을 다투게 두는 것보다 단순하고 교착이 없다.
2. 바뀐 파일·함수: `controller.py` `_route`, `_inflate`, `_reservations`, `_penalty_for`, `_replan_if_stuck`, `_check_deadlock`, `_sidestep`, `_after_sidestep`, `_admitted`, `_in_service`, `_plan_wait`, `_mark_blocked`, `_take_ahead`, `_note_block_report`, `_follow`; `planner.py` `astar`(거리 따로), `_rest_of_line`; `cells.py` `held_by_others`; `tools/compare_floor.py` `run`, `main`.
3. 핵심 코드

   ```python
   n = math.ceil(self._body / step) + 1                      # 몸 반지름이 닿을 수 있는 칸 수
   self._reach = [(di, dj) for di in range(-n, n + 1) for dj in range(-n, n + 1)
                  if math.hypot(max(0, abs(di) - 1) * step,  # 두 칸 사각형 사이 x 거리
                                max(0, abs(dj) - 1) * step)  # y 거리
                  < self._body]                              # 몸 반지름보다 가까우면 막는다
   ...
   avoid = self._inflate(self._others_cells(m.name)) | extra # 남의 칸을 넓혀서 피한다
   ```

   ```python
   def _admitted(self, m, leg):
       for name in self._missions:                           # 다른 로봇마다
           if 접근선 칸을 쥐고 있으면: return False             # 선 위나 도크에 있다
           if 같은 선으로 가는 중(대기 아님)이고 격리 안 됐으면: return False
       return 나보다 먼저 기다린 로봇이 없으면 True           # 먼저 온 순서
   ```

   ```python
   members, key = set(cycle), set(self._deadlock_key)
   same = now - self._deadlock_seen <= 1.0 and (            # 1초 안에 다시 보였고
       members <= key or (members & key and not self._deadlock_acted))  # 같은 무리면
   ...                                                       # 같은 일화: 유예 시간을 이어 센다
   turns = sorted(cycle, key=lambda name: (self._priority(name), name))  # 지금 순환만
   if not any(self._sidestep(self._missions[name], cycle) for name in turns):
       DEADLOCK_UNRESOLVED, 찾은 로봇을 기억                  # 나중에 그 로봇만 격리
   ```

   ```python
   if m.goal is None and m.robot_state not in ON_A_GOAL + (BLOCKED,):
       self._active[m.name].clear()                          # 정말 섰을 때만 옛 경로 보호를 푼다
   ...
   def _take_ahead(self, m):                                 # 막힘 표시는 goal이 다 끝난 뒤에
       if m.goal is not None or self._active[m.name]: return
       로봇이 쥔 앞 칸을 놓고 #blocked로 잡는다
   ```

4. 면접 한 문장: "칸 잠금으로 충돌은 막았지만 처음엔 세 대가 도크 앞에서 서로를 기다렸습니다. 원인은 남의 칸을 중심으로만 피하고 몸 반지름만큼 넓히지 않은 것과 도크 접근선을 여러 대가 다툰 것이었고, 형상 공간 팽창과 접근선 줄 세우기로 비켜 서기 없이 끝나게 했습니다."

### P3에서 하지 않은 것

- 시선 지름길은 예약 벌점을 보지 않는다(위 5-나). 시간 벌점 A*는 시각을 상태로 두지 않는다(재검수 4).
- 조용히 선 로봇 감시(`_watch_idle`)는 자유바닥에서 돌지 않는다(P4 몫).
- ROS 노드(`fleet_manager`) 시험은 P3 변경이 `fleet_core`뿐이라 다시 돌리지 않았다. 노드 판정기는 레인 기준이라 자유바닥은 머리 없는 실행으로만 확인했다.
