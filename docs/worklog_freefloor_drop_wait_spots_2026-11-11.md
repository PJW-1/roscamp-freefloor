# 작업 기록: 지도에 적는 대기 자리(wait_spots) 폐기 (2026-11-11 저녁)

브랜치 `chore/drop-wait-spots`, 출발점 `feat/slim-locks` 9f80154. push 하지 않았다.
사용자 지시: "지금 대기자리 코드는 폐기해. 양방향으로 들어가게 안 할 거니까". 도크를 한쪽으로 들어가 반대쪽으로
나가는 배치(시험 배치 arena_v6)로 가므로 지도에 적는 대기 자리가 필요 없다.

**결론부터.**

1. 커밋 5d8355e가 넣은 `wait_spots` 기능을 지웠다: 바닥 지도 키 읽기와 검사, 관제기의 `_wait_named`와 그 호출
   두 줄, arena_v5·arena_v6 지도의 `wait_spots:` 블록, 그 기능만 보는 테스트. 이제 지도에 `wait_spots` 키가 남아
   있어도 읽지 않는다(FloorMap은 모르는 키를 거부하지 않는다).
2. 대기 로봇은 예전처럼 `_plan_wait`가 접근선 시작에서 0.3–1.2 m 안의 빈 자리를 스스로 고른다(66dd298의
   "도크 출구를 막는 자리는 건너뛴다"를 거친다).
3. **arena_v6**: 헤드리스 전체 주행 씨앗 1–4 모두 주문 8건 완료, 불변식 위반 0, 칸 위반 0, 격리 0, 풀리지 않은
   교착 0. 끝난 시각은 지우기 전과 같거나 0.1–2.1초 빠르다. 기다린 시간 합은 씨앗 2(2.8 → 4.6 s)와 씨앗 4
   (1.3 → 2.6 s)에서 늘었다. 최대 틱 시간이 0.17–0.23 s에서 0.39–0.73 s로 늘었다(벽시계 값, 아래 표 참고).
4. **arena_v5**: 지우기 전 30개 모두 통과. 지운 뒤(테스트 파일을 고치기 전 그대로 돌림) 26 통과, 4 실패. 실패한
   넷은 모두 지도에 적은 대기 자리를 확인하는 테스트라 지웠다. 전체 주행(씨앗 1–3, 위치 오차 0, 주문 16건),
   통로 섞어 다니기, 선 따라 도크 들어오기, 66dd298의 대기·출구 테스트, 작업 도크를 떠나는 로봇 테스트는 지운 뒤에도
   통과했다. 단 헤드리스 결과다: arena_v5 Gazebo에서는 지도 대기 자리가 없을 때 로봇이 두 번 갇혔다(아래 5절).

## 1. 지운 것

| 파일 | 지운 것 |
|---|---|
| `pc/fleet_core/fleet_core/traffic/floor/controller.py` | `_plan_wait` 안 `named = self.floor.wait_spots.get(...)`와 `_wait_named` 호출(옛 1145–1147행), `_wait_named` 메서드(옛 1184–1208행) |
| `pc/fleet_core/fleet_core/traffic/floor/floor_map.py` | 모듈 설명글의 `wait_spots` 두 줄(옛 20–21행), `self.wait_spots = ...`(옛 119–120행), `_read_wait_spots`(옛 240–258행), `_check`의 대기 자리 빈 칸 검사(옛 334–337행) |
| `pc/fleet_core/fleet_core/data/floor_map_arena_v5.yaml` | `wait_spots:` 블록과 설명 주석(옛 46–54행) |
| `pc/fleet_core/fleet_core/data/floor_map_arena_v6.yaml` | `wait_spots:` 블록과 설명 주석(옛 45–49행) |
| `pc/fleet_core/test/test_floor_map.py` | `wait_spots` 절 전체(옛 595–623행): `waiting_data`, 테스트 4개(매개변수 포함 8개 실행) |
| `pc/fleet_core/test/test_floor_arena_v5.py` | 테스트 5개: `test_the_map_names_the_wait_spots_between_the_islands_and_on_the_parking_spots`, `test_a_robot_on_a_wait_spot_holds_no_cell_of_a_dock_line_and_its_body_no_other_spot`, `test_a_robot_between_the_islands_leaves_three_clear_rows_beside_it_north_and_south`, `test_robots_waiting_for_the_outbound_dock_stand_on_the_wait_spots_of_the_map`, `test_a_robot_called_to_a_busy_work_dock_waits_on_its_parking_spot`, 그리고 이들만 쓰던 `CROSS`, `spot_of`. `unnamed_floor()`(바닥 복사 + `wait_spots = {}`)와 `dock_wait_world`의 `floor` 인자를 없애고 `FLOOR`를 쓴다(이제 같은 바닥이다). `import copy` 삭제 |

`test_a_robot_between_the_islands_...`는 지운 뒤에도 통과했지만 지도 대기 자리 `CROSS`(두 섬 사이)의 기하만 보는
테스트라 함께 지웠다. `test_a_robot_called_to_a_busy_work_dock_waits_on_its_parking_spot`은 지도 대기 자리(P-2)에
그대로 서는지를 본다. 지운 뒤 pinky2는 (1.275, 0.315)에 섰다(작업 도크 선 시작 (1.55, 0.17)에서 약 0.31 m).

손대지 않은 것: `docs/freefloor/arena_v5/draw_layout.py`(그림 스크립트, `fl.get("wait_spots") or {}`라 키가 없어도
돈다), 지난 작업 기록 `docs/worklog_*.md`. `docs/관제_동작.md`에는 `wait_spots` 언급이 없었다. 5d8355e가 함께
바꾼 arena_v5의 `work_in` 선(남쪽 벽을 따라 서쪽에서 들어감)과 지도 주석의 나머지는 그대로 두었다.

## 2. 지우기 전후 숫자

헤드리스 `tools/compare_floor.run("arena_v6", ..., seed, "freefloor")`, `test_floor_arena_v6.py`의 `scenario()`와
`LOCKS`(주문 8건, 20초 간격, 위치 잡음 0.03 m, 속도 배율 5). 스크립트는 저장소 밖(scratchpad)의 `measure_v6.py`.
지우기 전은 같은 worktree에서 고치기 전(9f80154)에 쟀다.

| 씨앗 | 끝난 시각 end_s 전 → 후 | 마지막 출고 finish_s 전 → 후 | wait_total_s 전 → 후 | min_gap_m 전 → 후 | 최대 틱 s 전 → 후 |
|---|---|---|---|---|---|
| 1 | 198.4 → 197.7 | 190.0 → 189.3 | 2.3 → 2.3 | 0.290 → 0.290 | 0.219 → 0.517 |
| 2 | 198.9 → 196.9 | 190.5 → 188.5 | 2.8 → 4.6 | 0.287 → 0.274 | 0.234 → 0.726 |
| 3 | 197.2 → 197.1 | 188.8 → 188.7 | 1.0 → 1.0 | 0.290 → 0.290 | 0.173 → 0.394 |
| 4 | 197.9 → 195.8 | 189.5 → 187.4 | 1.3 → 2.6 | 0.290 → 0.273 | 0.178 → 0.729 |

전후 모두 네 씨앗에서 finished True, 출고 8, unresolved 0, isolations 0, deadlocks 0, sidesteps 0, 불변식 위반 0,
칸 위반 0. 테스트의 기준(min_gap ≥ 0.22 m, 최대 틱 < 1.0 s)은 모두 넘는다.

최대 틱은 벽시계 시간이라 기계 부하에 따라 달라진다. 전후 모두 다른 pytest 하나와 함께 돌렸다(조건은 비슷하다).
늘어난 것이 스스로 대기 자리를 찾는 비용(`_wait_route`, `_closes_way`의 너비 우선 탐색) 때문인지는 따로 재지 않았다.
지도 대기 자리가 있을 때는 그 탐색을 건너뛰었으니 그럴 가능성이 있다[추측].

arena_v5 `test/test_floor_arena_v5.py`:

| | 통과 | 실패 |
|---|---|---|
| 지우기 전(9f80154) | 30 | 0 |
| 지운 뒤, 테스트 파일 고치기 전 | 26 | 4 |
| 지운 뒤, 테스트 5개 지운 파일 | 전체 테스트 실행에서 확인(보고 참고) | |

지운 뒤 실패한 넷: `test_the_map_names_the_wait_spots_...`, `test_a_robot_on_a_wait_spot_holds_no_cell_...`,
`test_robots_waiting_for_the_outbound_dock_stand_on_the_wait_spots_of_the_map`(셋 다 `AttributeError: 'FloorMap'
object has no attribute 'wait_spots'`), `test_a_robot_called_to_a_busy_work_dock_waits_on_its_parking_spot`(대기
자리 (1.275, 0.315), 기대 P-2 (1.86, 0.87)).

## 3. 남긴 것: 66dd298

66dd298(같은 날, `wait_spots`보다 먼저)은 지도 대기 자리와 따로 돈다. 사용자가 지울지 따로 정할 일이라 그대로 두었다.

- `_plan_wait`가 대기 자리를 고를 때 `_closes_way`로 "이 자리에 서면(다른 대기 로봇 자리와 함께) 접근선 끝(도크)에서
  지금 닿는 주차 지점이나 다른 스테이션 선 시작 중 하나라도 못 닿게 되나"를 본다(`_way_goals_of`가 목표 칸,
  `_way_reach`가 빈 칸 너비 우선 탐색). 막는 자리는 건너뛰고(`WAY_TRIES`개까지 보고), 모두 막으면 예전 자리를 쓴다.
  `_wait_route`는 그 후보들 중 첫 구간을 출발할 수 있는 자리로 가는 길을 고른다.
- `_check_deadlock`에 `_line_waits`를 더한다: 대기 자리에 선 접근선 대기 로봇은 그 선 칸을 쥐었거나 그 선으로 가는
  로봇을 기다리는 것으로 친다. 그래서 "대기 로봇이 출구를 막고, 나가는 로봇은 선을 쥔 채 갇힌" 순환이 교착으로
  보이고 비켜 서기로 풀린다. `_may_get_by`는 막힌 지 `floor_pass_wait_s`가 안 됐고 다른 길을 찾는 중인 로봇을 그
  순환에서 뺀다(잠깐 생겼다 풀리는 순환을 알리지 않는다).

`wait_spots`를 지운 지금은 모든 대기 자리를 이 코드가 고르므로, 이것은 지도 대기 자리 없이 쓰이는 코드다. 지도
대기 자리가 있을 때는 `_wait_named`가 먼저 자리를 정해 이 탐색을 건너뛰었다. 도크를 지나쳐 나가는 배치(arena_v6)에서
도크 출구가 대기 로봇에 막힐 일이 줄었는지는 따로 보지 않았다. arena_v5의 66dd298 테스트 다섯(출구 테스트,
나갈 길 테스트, 막는 자리만 있을 때 테스트, 차례 대기 순환 2개)은 지운 뒤에도 통과했다.

## 4. 돌린 것

- `pc/fleet_core` 테스트(파일을 셋으로 나눠 pytest 셋): 결과는 커밋 보고에 적었다.
- arena_v6 헤드리스 씨앗 1–4(위 표), arena_v5 테스트 파일 전후.

## 5. 돌리지 않은 것, 확인하지 못한 것

- Gazebo, ROS 노드, 실물 로봇은 띄우지 않았다. `pc/fleet_manager` 테스트는 돌리지 않았다.
- arena_v5는 지도 대기 자리 없이 Gazebo에서 두 번 멈췄다(`docs/worklog_freefloor_arena_v5_2026-11-11.md` 2차·3차:
  출고 도크를 떠나는 로봇, 작업 도크를 떠나는 로봇이 대기 로봇에 갇힘). 3차는 66dd298을 넣은 뒤였고, 그 다음
  지도 대기 자리(5d8355e, 작업 도크 선을 남쪽 벽을 따라 옮긴 것과 함께)를 넣은 4차가 합격했다. 지운 지금 헤드리스
  테스트는 통과하지만 Gazebo에서 다시 멈추지 않는지는 모른다.
- arena_v6의 씨앗 5 이상, 주문 16건 같은 다른 시나리오는 돌리지 않았다.
- 틱 시간이 늘어난 원인은 재지 않았다.
