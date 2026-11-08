# pinky_nav_config

핑키 한 대의 Nav2 설정(파라미터, BT, launch, RViz). 핑키 Pi에서 namespace `/{pinky}`로 돈다. 지킬 약속은 Drive `핑키_FSM_Nav2_구현_안내_2026-11-07.md`와 `핑키_제어파트_전달사항_2026-11-06`(R4, R12~R17)이다.

| 파일 | 내용 |
|---|---|
| `params/nav2_params.yaml` | 제조사 `pinky_navigation` 설정(pinky_pro 014a09f) 위에 전달사항의 nav2_overrides 값을 덮었다. 바꾼 줄마다 이유가 있다 |
| `bt/plan_once_no_recovery.xml` | `navigate_to_pose`용 BT. 목표마다 한 번 계획하고 따라간다. 복구(Spin, BackUp)와 주기적 재계획이 없다(R14) |
| `launch/nav2.launch.py` | 지도 서버 둘, AMCL, controller, planner, behavior, bt_navigator, lifecycle manager 둘 |
| `rviz/pinky_nav.rviz` | 지도, 로봇, 라이다, costmap, AMCL 입자, 따라가는 경로. `<robot_namespace>`는 띄울 때 바꾼다 |

## 구성

- 모든 노드가 `/{namespace}` 안에서 돈다. TF는 `/{namespace}/tf`, `/{namespace}/tf_static`이고 프레임 이름(`map`, `odom`, `base_footprint`, `rplidar_link`)에는 접두사가 없다(R17).
- controller_server(`follow_path`, RPP)가 `cmd_vel`로 바로 낸다. velocity_smoother와 collision_monitor는 없다. `pinky_fsm`이 `follow_path`를 바로 부른다.
- AMCL은 1 cm 지도(`map_fine`), costmap은 5 cm 지도(`map`)를 읽는다(R15).
- local costmap은 0.025 m이고 `always_send_full_costmap: true`다. `pinky_fsm`이 `local_costmap/costmap`으로 BLOCKED를 판단한다.
- 버전 2를 막지 않으려고 둔 것(안내 3절): planner_server(`compute_path_to_pose`, `is_path_valid`), 전역 costmap의 라이다 층, behavior_server의 BackUp(어느 BT도 부르지 않음), controller의 `speed_limit` 토픽.
- costmap 층은 Jazzy에서 상대 토픽을 costmap 노드 안으로 풀기 때문에, 파라미터 파일에 `<robot_namespace>/scan`처럼 쓰고 launch가 `/pinky1/scan`으로 바꾼다.

## 실행

```bash
ros2 launch pinky_nav_config nav2.launch.py namespace:=pinky1 \
    map:=<5cm 지도.yaml> map_fine:=<1cm 지도.yaml> \
    initial_x:=0.35 initial_y:=1.55 initial_yaw:=1.5708
```

`use_composition:=true`를 주면 노드 아홉을 컨테이너(`component_container_isolated`) 한 프로세스에 올린다. 한 PC에 Gazebo 핑키 여럿을 띄우면 LOCALHOST discovery가 볼 수 있는 프로세스 수(약 32)를 넘기 때문이다. 이름, 파라미터, TF remap은 같고 기본값(false)은 노드마다 프로세스다.

실물 지도는 아직 없다(SLAM 뒤에 넣는다). Gazebo 지도와 한 번에 띄우는 방법은 `sim/fleet_gz_sim/README.md`.

## 시험

```bash
cd pinky/pinky_nav_config && python3 -m pytest -q test   # 설정 값 검사, ROS 필요 없음
```

`test/test_nav2_launch.py`(launch 구성 검사)는 ROS 2와 설치된 이 패키지가 있어야 돌고, 없으면 건너뛴다. `colcon test`로 돌린다.

`params/`의 시작값은 0.33 m 통로 Gazebo 연구에서 온 값이므로 실제 지도에서 다시 맞춘다.
