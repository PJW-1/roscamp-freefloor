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
