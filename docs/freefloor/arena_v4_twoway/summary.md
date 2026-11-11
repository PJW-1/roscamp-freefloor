# arena_v4 대 arena_v4_twoway 머리 없는 측정 (2026-11-11)

`tools/measure_twoway.py`로 만든 표다(`--summary runs_<구성>.csv`). 가짜 세계(FloorWorld), 속도 0.15 m/s,
위치 잡음 0.03 m, 잠금 값은 test_floor_arena_v4.py의 LOCKS, `floor_shortcut` 끔. 실행마다 한 줄씩 CSV에 있다.
"둘 다 완료 평균 끝"은 두 지도 모두 끝낸 씨앗만의 평균이다. 통로 통과 수는 로봇 실제 자리를 양방향 지도의
통로 가운데 선에 비추어 센 것이다(두 지도 같은 선, fwd = 지도에 적은 점 순서 방향).
Gazebo와 실물에서는 돌리지 않았다.

## 기본(standard)

| 구성 | 지도 | 판 | 완료 | 둘 다 완료 평균 끝(s) | 대기 순환 | 못 풂 | 격리 | 칸 위반 | 최소 간격(m) |
|---|---|---|---|---|---|---|---|---|---|
| standard 2x10 | arena_v4 | 6 | 5 | 317.9 (n=5) | 0 | 0 | 0 | 1 | 0.275 |
| standard 2x10 | arena_v4_twoway | 6 | 5 | 307.9 (n=5) | 0 | 0 | 0 | 1 | 0.275 |
| standard 3x16 | arena_v4 | 6 | 6 | 369.7 (n=6) | 0 | 0 | 0 | 0 | 0.269 |
| standard 3x16 | arena_v4_twoway | 6 | 6 | 370.9 (n=6) | 10 | 0 | 0 | 0 | 0.235 |
| standard 3x6 | arena_v4 | 12 | 12 | 154.7 (n=12) | 0 | 0 | 0 | 0 | 0.277 |
| standard 3x6 | arena_v4_twoway | 12 | 12 | 153.6 (n=12) | 0 | 0 | 0 | 0 | 0.277 |

| 지도 | gap23_back | gap23_fwd | mid_n_back | mid_n_fwd | mid_s_back | mid_s_fwd | north3_back | north3_fwd | north4_back | north4_fwd | south4_back | south4_fwd |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| arena_v4 | 0 | 218 | 0 | 436 | 0 | 218 | 218 | 0 | 0 | 218 | 218 | 0 |
| arena_v4_twoway | 0 | 218 | 0 | 430 | 6 | 218 | 218 | 0 | 0 | 212 | 218 | 6 |

## 비대칭(asym, 3대 8건)

| 구성 | 지도 | 판 | 완료 | 둘 다 완료 평균 끝(s) | 대기 순환 | 못 풂 | 격리 | 칸 위반 | 최소 간격(m) |
|---|---|---|---|---|---|---|---|---|---|
| asym p10_i4_s30 | arena_v4 | 3 | 3 | 328.2 (n=3) | 4 | 2 | 0 | 0 | 0.234 |
| asym p10_i4_s30 | arena_v4_twoway | 3 | 3 | 326.9 (n=3) | 4 | 0 | 0 | 0 | 0.229 |
| asym p10_i4_s8 | arena_v4 | 3 | 3 | 198.7 (n=3) | 0 | 0 | 0 | 0 | 0.270 |
| asym p10_i4_s8 | arena_v4_twoway | 3 | 3 | 196.4 (n=3) | 0 | 0 | 0 | 0 | 0.276 |
| asym p10_i8_s4 | arena_v4 | 3 | 3 | 280.6 (n=3) | 0 | 0 | 0 | 0 | 0.268 |
| asym p10_i8_s4 | arena_v4_twoway | 3 | 3 | 285.5 (n=3) | 2 | 0 | 0 | 0 | 0.252 |
| asym p30_i4_s30 | arena_v4 | 3 | 3 | 328.2 (n=3) | 6 | 0 | 0 | 0 | 0.234 |
| asym p30_i4_s30 | arena_v4_twoway | 3 | 3 | 326.9 (n=3) | 6 | 0 | 0 | 0 | 0.237 |
| asym p30_i4_s8 | arena_v4 | 3 | 3 | 254.9 (n=3) | 0 | 0 | 0 | 0 | 0.284 |
| asym p30_i4_s8 | arena_v4_twoway | 3 | 3 | 254.0 (n=3) | 0 | 0 | 0 | 0 | 0.284 |
| asym p30_i8_s4 | arena_v4 | 3 | 3 | 280.1 (n=3) | 0 | 0 | 0 | 0 | 0.284 |
| asym p30_i8_s4 | arena_v4_twoway | 3 | 3 | 274.1 (n=3) | 0 | 0 | 0 | 0 | 0.283 |

| 지도 | gap23_back | gap23_fwd | mid_n_back | mid_n_fwd | mid_s_back | mid_s_fwd | north3_back | north3_fwd | north4_back | north4_fwd | south4_back | south4_fwd |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| arena_v4 | 0 | 144 | 0 | 288 | 0 | 144 | 144 | 0 | 0 | 144 | 144 | 0 |
| arena_v4_twoway | 3 | 144 | 0 | 269 | 7 | 135 | 141 | 0 | 9 | 137 | 135 | 7 |

## 장애물(obstacle, 3대 6건)

| 구성 | 지도 | 판 | 완료 | 둘 다 완료 평균 끝(s) | 대기 순환 | 못 풂 | 격리 | 칸 위반 | 최소 간격(m) |
|---|---|---|---|---|---|---|---|---|---|
| obstacle mid_0.97_0.85 | arena_v4 | 3 | 0 | - (n=0) | 5 | 0 | 3 | 0 | 0.234 |
| obstacle mid_0.97_0.85 | arena_v4_twoway | 3 | 0 | - (n=0) | 81 | 5 | 3 | 0 | 0.245 |
| obstacle north3_0.78_0.905 | arena_v4 | 3 | 0 | - (n=0) | 5 | 0 | 3 | 0 | 0.234 |
| obstacle north3_0.78_0.905 | arena_v4_twoway | 3 | 0 | - (n=0) | 0 | 0 | 3 | 0 | 0.257 |

| 지도 | gap23_back | gap23_fwd | mid_n_back | mid_n_fwd | mid_s_back | mid_s_fwd | north3_back | north3_fwd | north4_back | north4_fwd | south4_back | south4_fwd |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| arena_v4 | 0 | 0 | 0 | 1 | 0 | 6 | 0 | 0 | 0 | 0 | 12 | 0 |
| arena_v4_twoway | 3 | 0 | 0 | 0 | 0 | 12 | 2 | 0 | 2 | 0 | 15 | 0 |
