# 자유바닥 교착 스트레스 작업 기록 (2026-11-11 밤, 중간에 멈춤)

브랜치 `fix/deadlock-stress` (시작점 `integrate/night-1111` f98dae9). 사용량 한도로 중간에 멈췄다.
이 문서만 읽고 어디까지 했는지 알 수 있게 적는다.

## 결론

- 고친 것 하나: **마주 오는 로봇 앞에서 일찍 서기**(`_halt_head_on`, 스위치 없음, 기본 동작).
  arena_v6 swap에서 끝나지 않던 쌍 7개(스위치 끔 4개, `floor_keep_right=true` 3개)가 이 수정 뒤 모두
  끝난다(새 테스트 7개, 고치기 전 7개 모두 실패 → 고친 뒤 통과).
- 캠페인(`swap` 전체, `cross`, `ring`)은 **고친 뒤에 돌리지 못했다**. 아래 "고치기 전" 숫자만 있다.
- 할 일 1(우측통행 통로 판정을 m 기준으로), 3(`_plan_wait` 속도)은 따로 작업자 브랜치에서 진행했고
  이 브랜치에는 **합치지 않았다**(아래 "다른 브랜치").
- `pc/fleet_core` 전체 테스트는 **돌리지 않았다**.

## 상황별 표 (arena_v6, 위치 잡음 0.03 m)

| 상황 | 스위치 | 고치기 전(62c82e6) | 고친 뒤 |
|---|---|---|---|
| swap 55쌍 | 끔 | 51 ok / 4 stuck (NA<>QO, MC<>SE, NC<>QO, SE<>WS) | 실패하던 4쌍 모두 ok(테스트). 55쌍 전체는 안 돌림 |
| swap 55쌍 | `floor_keep_right=true` | 52 ok / 3 stuck (NA<>QO, NC<>QO, QO<>SC), 나란히 지나가기 0번 | 실패하던 3쌍 모두 ok(테스트). 전체는 안 돌림 |
| cross(씨앗 1~3, 각 120판) | 끔/켬 | 안 돌림 | 안 돌림 |
| ring 120판 | 끔/켬 | 안 돌림 | 안 돌림 |

고치기 전 CSV: `/tmp/claude-1000/-home-ane-ros2camp/6f858ace-77ac-4537-9adb-52656f20da9d/scratchpad/stress/keep/swap_fix1_*.csv`, `swap_kr_*.csv`.
broken(불변식 위반)은 고치기 전 swap에서 0. 고친 뒤 캠페인이 없어 캠페인 broken은 모른다
(새 테스트 7판은 `result == "ok"`라 broken 아님).

## 고친 것: 마주 오는 로봇 앞에서 일찍 서기

- 원인(확인, NA<>QO 추적): 두 로봇의 굴러가는 잠금 끝이 4.5 s에 만났다(중심 간격 0.49 m). 각자
  잠근 끝까지 MoveTo를 마저 따라가 중심 0.196 m에 섰다. 대기 순환은 둘 다 선 5.9 s에야 보였고,
  `_head_on`은 MoveTo가 없는(m.goal None) 로봇을 마주 달리는 것으로 치지 않아 유예 5 s를 기다렸다.
  그 뒤 비켜 서려 해도 0.2 m 간격에서는 제자리에서 돌 원을 잠글 수 없어 "비켜 설 곳 없음"
  (DEADLOCK_UNRESOLVED) → 30 s 뒤 격리.
- 수정(`controller.py`):
  - `_halt_head_on`: 잠금이 늘지 않았고 기다리는 칸의 주인이 길 위에서 마주 보는 로봇(`_facing`)이면
    MoveTo를 취소해 지금 선다. `_Leg.halt_on`에 그 경로를 적고, 잠금이 늘면 지운다(새 경로를 받으면
    저절로 풀린다).
  - `_send`: 일찍 선 다리는 잠금이 늘 때까지 다시 보내지 않는다.
  - `_replan_if_stuck`: 일찍 선 다리는 앞이 짧은 것으로 친다(다시 계획하기가 그대로 돈다).
  - `_head_on` → `_facing(a, b, moving)`으로 나눴다. 일찍 선 로봇은 마주 달리는 것으로 쳐서 유예 없이
    비켜 서기로 간다.
  - 고친 뒤 NA<>QO: 4.4 s에 pinky1이 0.50 m 간격에서 서고, 4.9 s 순환 감지, 5.0 s 가운데 통로
    (0.98, 0.52)로 비켜 섬, 11.0 s / 16.9 s 둘 다 도착.
- 지키는 테스트: `pc/fleet_core/test/test_floor_head_on.py`의
  `test_robots_that_would_end_nose_to_nose_stop_early_and_both_arrive`(4쌍),
  `test_the_same_with_keep_right`(3쌍).
- 안전: 잡은 칸은 그대로 쥐고 덜 가는 것뿐이다(취소한 MoveTo의 칸은 결과가 올 때까지 쥔다).
  불변식 검사, 잠금 폭, 지도 여유 값은 건드리지 않았다.

## 테스트 결과 (나온 그대로)

- 새 테스트 7개, 고치기 전: `7 failed, 5 deselected in 251.49s`.
- `test_floor_head_on.py` 전체, 고친 뒤(`halted: bool` 판): `12 passed in 151.96s`.
- 그 뒤 `halted: bool`을 `halt_on`(경로 객체)으로 바꿔 새 경로에서 저절로 풀리게 했다. 이 판으로는
  head_on 테스트를 **다시 돌리지 않았다**.
- 사건·명령 비교 테스트(`test_floor_retreat.py`의 `BASE_DIGESTS`, `test_floor_obstacle_report.py`):
  `test_floor_retreat.py test_floor_obstacle_report.py` 함께 `42 passed in 934.77s`(이것도 `halted: bool`
  판으로 시작한 실행이다. `halt_on` 판으로는 다시 돌리지 않았다). arena_v4 비교 테스트
  (`test_floor_arena_v4.py`)는 안 돌림.
- 전체 `pc/fleet_core` 테스트: 안 돌림.

## 다른 브랜치(이 브랜치에 안 합침)

- `_plan_wait` 속도(할 일 3): 작업자 worktree `.claude/worktrees/agent-a9ee6a1eaa943d559`, 브랜치
  `fix/plan-wait-speed`(f98dae9에서), 커밋 8c29c61. `_plan_wait`의 후보 미리 거르기를 가까운 순서로
  하나씩 검사하는 제너레이터(결과 캐시)로 바꿨다. 새 테스트 `test_wait_spot_search_checks_only_the_cells_it_needs`
  (`test_floor_arena_v6.py`): 한 번의 `_standing_cells` 호출 수 고치기 전 1283회(실패) → 2회(통과).
  결과가 같다는 것은 코드 읽기로만 확인했고 v4/v5/v6, digest, 대기 자리 테스트와 Codex 리뷰는
  **못 돌렸다**(그 작업자 보고). 그 시험 전에는 합치지 말 것.
- 우측통행 통로 판정 m 기준(할 일 1): 작업자 worktree `.claude/worktrees/agent-aca6c322efa5adefe`,
  브랜치 `fix/keep-right-meters`(f98dae9에서). 결과는 그 작업자의 보고를 볼 것.

## 남은 것, 돌리지 않은 것

- 고친 뒤 캠페인 전부(swap 55, cross 씨앗 1~3 각 120판, ring 120판 × 스위치 끔/켬), arena_v5 캠페인.
- 고치기 전 cross, ring 숫자(기준선).
- 좁은 한 줄 통로에서 마주친 두 대가 1.2 m 안에 비켜 설 곳이 없을 때의 물러나기(`_retreat` 활용)는
  손대지 않았다. 캠페인으로 그런 실패가 남는지부터 봐야 한다.
- Codex 리뷰: 이 수정에 대해 돌리지 못했다(한도로 멈춤).
- 전체 `pc/fleet_core` 테스트.

## 다시 돌리는 명령

```
# 판 하나 추적
python3 /tmp/claude-1000/-home-ane-ros2camp/6f858ace-77ac-4537-9adb-52656f20da9d/scratchpad/me/dtrace.py <저장소> arena_v6 NA QO 4 9 60
# 캠페인(스위치 끔/켬 × swap, cross1~3, ring, 셋으로 나눠 동시 3개)
bash /tmp/claude-1000/-home-ane-ros2camp/6f858ace-77ac-4537-9adb-52656f20da9d/scratchpad/me/campaign.sh <저장소> <출력 폴더> 3 arena_v6 swap cross1 cross2 cross3 ring
python3 /tmp/claude-1000/-home-ane-ros2camp/6f858ace-77ac-4537-9adb-52656f20da9d/scratchpad/me/summary.py <출력 폴더> show
# 새 테스트
cd pc/fleet_core && python3 -m pytest test/test_floor_head_on.py -q
```
고치기 전 트리는 `git archive f98dae9`로 `/tmp/claude-1000/-home-ane-ros2camp/6f858ace-77ac-4537-9adb-52656f20da9d/scratchpad/me/base`에 풀어 두었다(기준선 캠페인용).
