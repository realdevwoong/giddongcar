# pinky_driving

단일 Pinky Pro에서 카메라 기반 흰 테이프 인식과 감독형 라인 추종을 실험합니다. 현재 구현은 `.pt` 모델이나 곡선 모델 대신 HSV 흰색 마스크와 Hough 선분 검출을 사용해 좌우 직선 테이프를 찾고, 선분의 기울기로 교차보도 가로줄을 걸러냅니다. 이 방식은 직선 코스용 관찰 프로토타입이며 카메라 BLE/HTTP 연결은 `pinky_fleet`의 모듈을 재사용합니다.

## 관찰 모드

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/giddongcar/pinky_pro/install/setup.bash
export ROS_DOMAIN_ID=15 ROS_STATIC_PEERS=192.168.0.6
export PYTHONPATH="$HOME/Desktop/giddongcar/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"

ros2 run pinky_driving tape_lane_drive --robot-ip 192.168.0.6 --mode observe
```

로봇2는 IP를 `192.168.0.8`, ROS domain을 `17`로 바꿉니다. Pinky Studio 및 대시보드 카메라 스트림은 끄고 실행하세요. 관찰 창은 흰색 마스크, 좌우 직선, 중앙 경로, 검출 신뢰도와 중심 오차를 표시합니다. `s`는 원본/오버레이/흰색 마스크/설정 JSON을 `~/vision_drive_observations`에 저장하고, `q` 또는 Ctrl+C는 종료합니다.

### 현장 영상에 맞춰 조정

| 인자 | 기본값 | 용도 |
|---|---:|---|
| `--white-value` | `165` | 테이프 밝기 기준. 어두운 화면이면 낮춥니다. |
| `--max-saturation` | `115` | 흰색 판정의 최대 채도. 색 있는 바닥이 흰색으로 잡히면 낮춥니다. |
| `--roi-top` | `0.42` | 차선 검출 영역의 시작 높이 비율. 위쪽의 조명·배경과 교차보도 가로줄을 줄입니다. |
| `--lane-width-min` | `0.16` | 영상 너비 대비 최소 테이프 경계 간격 |
| `--lane-width-max` | `0.92` | 영상 너비 대비 최대 테이프 경계 간격 |

예: 그림자에서 테이프가 끊겨 보이면 밝기 기준을 조금 낮춰 관찰합니다. 교차보도 선이 경계로 잡히면 ROI를 더 아래로 옮기고, 실제 테이프가 잘리면 ROI를 위로 올립니다.

```bash
ros2 run pinky_driving tape_lane_drive --robot-ip 192.168.0.6 \
  --mode observe --white-value 145 --max-saturation 130
```

## 감독형 주행 실험

**현재 주행 기능은 실물 코스에서 검증되지 않았으므로, 먼저 관찰 모드에서 직선·커브·교차로 전체를 확인해야 합니다.** 차선 검출만으로 장애물·횡단보도를 인식하지 않습니다. 장애물 정지는 LiDAR `/scan`에 의존합니다.

주행 전에 다음을 확인해야 합니다.

- 로봇 측 `cmd_vel` watchdog과 물리 비상정지가 실제로 동작함
- Nav2/대시보드 등 다른 `/cmd_vel` 발행 프로세스가 종료됨
- LiDAR `/scan`이 해당 ROS domain에서 최신 값으로 들어옴
- 사람이 코스에 장애물이 없는 상태에서 로봇 바로 옆을 지키며 감독함

확인이 끝난 통제 구역에서만 낮은 속도로 실행합니다. 확인 플래그는 실제 검증을 대체하지 않습니다.

```bash
ros2 run pinky_driving tape_lane_drive --robot-ip 192.168.0.6 \
  --mode drive --enable-motion --confirm-supervised-test --watchdog-verified \
  --max-linear 0.02 --max-angular 0.15 --stop-distance 0.40
```

카메라/차선/라이다 입력이 끊기거나 차선 양쪽 경계가 충분히 잡히지 않으면 주기적으로 0 속도를 발행합니다. 좌우선 기울기 차이, 차선 폭의 변화, 적합 오차, 이전 프레임 대비 이동도 검사합니다. Ctrl+C로 정상 종료할 때도 0 속도를 보냅니다. 프로세스 강제 종료나 PC 전원 상실을 대신할 watchdog은 로봇에서 별도로 검증해야 합니다.
