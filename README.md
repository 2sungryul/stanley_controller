# Stanley Controller

ROS2 Jazzy용 Stanley path following 알고리즘 구현 패키지입니다. turtlesim에서 원형 경로를 자동으로 추종합니다.

## Stanley 알고리즘 개요

Stanley 알고리즘은 2005년 DARPA Grand Challenge에서 Stanford 대학팀이 사용한 경로 추종 알고리즘입니다.

### 주요 특징
- **Cross-track error**: 경로로부터의 수직 거리 오차 보정
- **Heading error**: 경로의 방향과 차량 방향의 차이 보정
- **속도 적응형**: 차량 속도에 따라 조향 각도 조정

### 조향 법칙
```
δ = θ_e + arctan(k * e / (k_s + v))
```
- δ: 조향 각도
- θ_e: Heading error (방향 오차)
- k: Cross-track error gain
- e: Cross-track error (수직 거리 오차)
- k_s: Softening constant
- v: 차량 속도

## 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/turtle1/pose` | turtlesim/msg/Pose | 거북이의 현재 위치와 방향 |
| `/desired_path` | nav_msgs/msg/Path | 추종할 목표 경로 |

## 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/turtle1/cmd_vel` | geometry_msgs/msg/Twist | 속도 명령 (선속도, 각속도) |

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| k_gain | double | 1.5 | Cross-track error gain (높을수록 경로로 빠르게 복귀) |
| k_soft | double | 0.5 | Softening gain (저속에서의 불안정성 방지) |
| max_linear_vel | double | 2.0 | 최대 선속도 (m/s) |
| min_linear_vel | double | 0.5 | 최소 선속도 (m/s) |
| max_angular_vel | double | 2.0 | 최대 각속도 (rad/s) |
| lookahead_distance | double | 0.5 | Lookahead 거리 (m) |
| goal_tolerance | double | 0.1 | 목표 도달 허용 오차 (m) |
| wheelbase | double | 0.1 | 차량 축간 거리 (m) |

## 빌드 방법

```bash
cd ~/turtle_tracker_ws
colcon build --packages-select stanley_controller
source install/setup.bash
```

## 실행 방법

### 방법 1: 전체 시스템 실행 (권장)

```bash
ros2 launch stanley_controller full_system.launch.py
```

이 명령어는 다음을 모두 실행합니다:
- turtlesim_node
- circular_path_publisher_node
- stanley_controller_node
- turtle_tracker_node

### 방법 2: 개별 실행

```bash
# 터미널 1: turtlesim
ros2 run turtlesim turtlesim_node

# 터미널 2: path publisher
ros2 run path_publisher circular_path_publisher_node

# 터미널 3: stanley controller
ros2 run stanley_controller stanley_controller_node

# 터미널 4: tracker (시각화)
ros2 run turtle_tracker turtle_tracker_node
```

### 방법 3: 커스텀 파라미터로 실행

```bash
ros2 launch stanley_controller stanley_controller.launch.py \
    k_gain:=2.0 \
    max_linear_vel:=1.5 \
    lookahead_distance:=0.8
```

## 파라미터 튜닝 가이드

### k_gain (Cross-track error gain)
- **높은 값 (2.0~3.0)**: 경로에서 벗어나면 빠르게 복귀하지만, 진동(oscillation) 발생 가능
- **낮은 값 (0.5~1.0)**: 부드러운 제어이지만 경로 추종 성능 저하
- **추천값**: 1.5

### k_soft (Softening constant)
- **높은 값 (1.0~2.0)**: 저속에서 더 안정적이지만 반응 느림
- **낮은 값 (0.1~0.5)**: 빠른 반응이지만 저속에서 불안정할 수 있음
- **추천값**: 0.5

### max_linear_vel (최대 선속도)
- **높은 값 (3.0~4.0)**: 빠른 추종이지만 정확도 감소
- **낮은 값 (1.0~1.5)**: 정확한 추종이지만 느림
- **추천값**: 2.0

### lookahead_distance (Lookahead 거리)
- **높은 값 (1.0~2.0)**: 부드러운 경로 추종, 급격한 커브에서 shortcutting 발생 가능
- **낮은 값 (0.2~0.5)**: 정확한 추종이지만 진동 발생 가능
- **추천값**: 0.5

## 알고리즘 작동 흐름

1. **가장 가까운 경로 점 찾기**: 현재 위치에서 가장 가까운 경로 상의 점 탐색
2. **Lookahead 포인트 선택**: 전방 예측 거리만큼 떨어진 목표 점 선택
3. **Cross-track error 계산**: 현재 위치에서 경로까지의 수직 거리
4. **Heading error 계산**: 현재 방향과 경로 방향의 차이
5. **조향 각도 계산**: Stanley 법칙을 사용하여 조향 각도 산출
6. **속도 명령 생성**: 계산된 조향 각도를 각속도로 변환
7. **cmd_vel 발행**: turtlesim으로 속도 명령 전송

## 성능 모니터링

### 로그 출력
컨트롤러는 5초마다 다음 정보를 출력합니다:
- **CTE**: Cross-track error (경로로부터의 거리)
- **Heading error**: 방향 오차 (도 단위)
- **Vel**: 현재 선속도
- **Ang_vel**: 현재 각속도

### 토픽 모니터링

```bash
# cmd_vel 확인
ros2 topic echo /turtle1/cmd_vel

# pose 확인
ros2 topic echo /turtle1/pose

# 발행 주기 확인
ros2 topic hz /turtle1/cmd_vel
```

### 파라미터 실시간 변경

```bash
# k_gain 조정
ros2 param set /stanley_controller k_gain 2.0

# 최대 속도 조정
ros2 param set /stanley_controller max_linear_vel 1.5

# 현재 파라미터 확인
ros2 param list /stanley_controller
ros2 param get /stanley_controller k_gain
```

## 시각화

turtle_tracker와 함께 사용하면 다음을 확인할 수 있습니다:
- **초록색 원**: 목표 경로
- **빨간색 선**: 거북이의 실제 궤적
- **파란색 화살표**: 거북이의 방향

## 문제 해결

### 거북이가 경로를 따라가지 않는 경우
1. 토픽이 제대로 발행되는지 확인
   ```bash
   ros2 topic list
   ros2 topic echo /desired_path
   ```
2. k_gain을 증가시켜보세요 (예: 2.0~3.0)
3. lookahead_distance를 줄여보세요 (예: 0.3~0.4)

### 거북이가 진동하는 경우
1. k_gain을 감소시켜보세요 (예: 0.8~1.2)
2. k_soft를 증가시켜보세요 (예: 1.0~1.5)
3. max_angular_vel을 제한해보세요 (예: 1.5)

### 거북이가 너무 느린 경우
1. max_linear_vel을 증가시켜보세요 (예: 3.0)
2. min_linear_vel을 증가시켜보세요 (예: 1.0)

### 경로를 크게 벗어나는 경우
1. lookahead_distance를 줄여보세요
2. k_gain을 증가시켜보세요
3. 제어 주기를 빠르게 하세요 (코드 수정 필요)

## 의존성

- ROS2 Jazzy
- rclcpp
- turtlesim
- nav_msgs
- geometry_msgs
- tf2
- tf2_geometry_msgs

## 참고 자료

- [Stanley Method Paper](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf)
- [Path Tracking Methods](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

## 패키지 구조

```
stanley_controller/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   └── stanley_controller_node.cpp
├── include/
│   └── stanley_controller/
└── launch/
    ├── stanley_controller.launch.py
    └── full_system.launch.py
```

## 향후 개선 사항

- [ ] 적응형 lookahead distance (속도에 따라 조정)
- [ ] 곡률 기반 속도 조정
- [ ] 경로 예측 기능
- [ ] 장애물 회피 기능
- [ ] PID 제어와의 비교 모드

## 라이선스

Apache-2.0
