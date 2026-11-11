# pinky_fsm

핑키 한 대의 이동·도킹 명령을 받아 Nav2로 넘기고 상태를 보고한다. 핑키 Pi에서 namespace `/{pinky}`로 돈다.

- 액션 서버: `move_to` (`MoveTo`), `dock` (`Dock`), `retreat` (`Retreat`, 제안. 핑키 계약 3절 10번). 이름과 QoS는 `interfaces/fleet_interfaces/contract.yaml`에서 읽는다.
- 발행: `state` (`RobotState`, 10 Hz, 변할 때만이 아니라 계속)
- 구독: `/fleet/heartbeat`(계약의 HEARTBEAT QoS), `odom`(속도), `local_costmap/costmap`(BLOCKED 판단)
- Nav2: controller의 `follow_path`(`FollowPath`)를 부른다. NavigateToPose는 부르지 않는다.
- 위치: TF `map` → `base_footprint`(AMCL). `pose_timeout_s`(1 s)보다 오래된 TF는 위치를 모르는 것으로 친다.

레인 그래프는 모른다. `pc/*`를 import하지 않는다(계약은 `pinky_fsm/contract.py`가 직접 읽는다). 지킬 약속은 `docs/구현_약속.md`, `docs/핑키_계약.md`, Drive `핑키_FSM_Nav2_구현_안내_2026-11-07.md`.

## 구조

| 파일 | 하는 일 |
|---|---|
| `pinky_fsm/core.py` | 판단 전부. ROS를 모르고 입력을 받아 명령(`SendPath`, `CancelNav`, `Finish`)을 돌려준다 |
| `pinky_fsm/node.py` | ROS와 잇기만 한다(액션 서버, FollowPath 클라이언트, TF, 토픽) |
| `pinky_fsm/geometry.py` | 레인 폴리라인을 0.05 m 간격 경로로 펼치기, 위치 투영, 남은 선 |
| `pinky_fsm/costmap.py` | local costmap에서 경로 앞 물체 찾기 |
| `pinky_fsm/contract.py` | 계약 파일에서 이름과 QoS 읽기 |

## 동작

| 상황 | 동작 | 근거 |
|---|---|---|
| `move_to`, path 있음 | 로봇 위치를 path에 투영해 가장 가까운 점부터 끝까지 0.05 m 간격으로 펼쳐 FollowPath로 보낸다. 마지막 점의 방향은 `target_pose.theta` | 안내 1절, 계약 2절, 전달사항 6.1 |
| `move_to`, path 비어 있음 | 지금 자리에서 `target_pose`까지 곧은 선을 FollowPath로 보낸다 | 전달사항 2.3(두 모드 모두) |
| 실행 중 새 `move_to` | 이전 목표는 `CANCELED`로 끝내고, FollowPath는 **취소하지 않고** 새 경로를 바로 보낸다. 로봇은 서지 않는다 | 안내 2.1, R6 |
| path가 로봇에서 0.15 m 넘게 떨어짐 | 따라가지 않고 `PATH_NOT_CONNECTED`로 실패. 이전 목표도 끝나므로 선다 | 안내 2.2, R20 |
| 취소 | FollowPath를 취소해 세우고 `CANCELED`로 끝낸다 | R7 |
| Nav2 성공 | 목표에서 0.15 m 안일 때만 성공. 멀면 남은 경로를 다시 보낸다(3번까지) | R4, 전달사항 6.1 goal_done |
| 경로 끝 0.15 m 안 | 0.5 s마다 남은 경로를 다시 보낸다 | R13 |
| Nav2 실패, 앞에 물체 없음 | `NAV_ABORTED` | R8 |
| FollowPath 거절(서버 없음 포함) | 남은 FollowPath를 모두 취소해 세우고 `NAV_ABORTED`. 교체 때 앞 것을 취소하지 않으므로 앞 것이 아직 달릴 수 있다 | 안내 2.1 |
| 이동 중 1 s 서 있고 경로 앞 0.5 m 안에 물체 | `BLOCKED`. Nav2가 물체 때문에 멈춰도 바로 끝내지 않고, 물체가 1 s 안 보이면 끄고 남은 경로를 다시 보낸다. 막힌 채 `block_timeout_s`(90 s, 제안)가 지나면 `TIMEOUT`(Nav2가 goal을 쥐고 있으면 세운다). 관제가 막힘 5 s(`blocked_mark_s`)에 goal을 먼저 취소하므로 관제가 취소하지 못했을 때의 마지막 안전장치다. 취소된 뒤에도 물체가 있으면 `BLOCKED` | 계약 4절, R9 |
| 목표 실행 중 heartbeat·목표 수락이 1.5 s 없음 | 끊긴 지 1.5 s에 FollowPath를 취소해 정지 명령을 내고(제동 거리는 별도) `HEARTBEAT_LOST`로 끝낸다. 벽시계로 잰다 | R10, 계약 3절 5번 |
| 위치를 모름 | `LOCALIZING`. 이때 온 목표는 `NAV_ABORTED`. 실행 중에 위치를 잃으면 FollowPath를 취소해 세우고 `NAV_ABORTED` | 계약 3절 6번 |
| `retreat`(제안) | 이동을 먼저 끝내고(`CANCELED`) FollowPath 결과가 오고 로봇이 선 뒤 물러난다. path가 비면 Nav2 BackUp으로 곧게, 있으면 그 경로를 후진 controller `reverse_controller_id`(기본 `FollowPathReverse`)로 따른다. 결과 reason은 `backup.reason_of` | 핑키 계약 3절 10번 |
| `dock` | **pinky_dock이 아직 없어** 이동을 멈추고 실패(`success=false`)를 돌려준다. 도킹을 했다고 꾸미지 않는다 | — |

"물체"는 local costmap 값이 99(INSCRIBED) 이상인 칸이다. Nav2 local costmap은 `always_send_full_costmap: true`여야 한다.

## 실행

```bash
ros2 launch pinky_fsm pinky_fsm.launch.py namespace:=pinky1
# 관제 없이 손으로 시험할 때는 heartbeat_timeout_s:=0
```

Gazebo에서 핑키 한 대를 Nav2와 함께 돌리는 방법은 `sim/fleet_gz_sim/README.md`.

## 시험

```bash
cd pinky/pinky_fsm && python3 -m pytest -q test   # ROS가 없으면 ROS 시험은 건너뛴다
```

ROS 시험은 `ROS_DOMAIN_ID`(6 아님)와 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`가 있어야 돈다. Nav2 대신 `test/fake_nav.py`(FollowPath를 받아 TF·odom을 내는 가짜 controller)를 쓴다. `test_ros_matches_fake_pinky.py`는 같은 목표 순서에 FakePinky와 결과·상태 값이 같은지 본다(PATH_NOT_CONNECTED와 dock은 차이가 알려져 있어 비교하지 않는다).
