# AGENTS.md

팀원과 AI 코딩 에이전트(Claude Code, Codex, Cursor)가 함께 따르는 규칙. 설치는 [README.md](README.md).

## 목표와 환경

공유기 1대 + 노트북 2대 + Pinky Pro 2대. 두 로봇이 **서로 부딪히지 않고** 지도 위를 주행하고, **카메라 탐지 결과**를 관제 화면에 띄운다. 지금은 Gazebo에서 먼저 검증하는 단계.
Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · Fast DDS(기본, Cyclone 전환은 미정).

## 어디에 무엇이 있나

| 경로 | 내용 |
|---|---|
| `pinky_pro/` | colcon 워크스페이스. 빌드는 항상 여기서 |
| `pinky_pro/src/pinky_*` | 제조사(pinklab) 원본. 꼭 필요할 때만 최소 수정하고 이유를 커밋에 적는다 |
| `pinky_pro/src/pinky_fleet/` | 팀 관제 패키지(PC 쪽: Nav2 2개, 웹 대시보드, 시뮬 launch). [README](pinky_pro/src/pinky_fleet/README.md) |
| `pinky_pro/src/pinky_gui/` | 지금은 쓰지 않음 |
| `jinho/` | pinky_fleet의 원본 작업 폴더. 작성자가 정리할 예정이라 수정하지 않는다 |
| `docs/` | 실행 방법과 컨벤션 상세 |

새 코드를 어느 패키지에 넣을지는 [docs/structure.md](docs/structure.md).

## 빌드·테스트

```bash
source /opt/ros/jazzy/setup.bash && cd ~/giddongcar/pinky_pro
colcon build --symlink-install --packages-select <패키지>   # 새 파일이나 entry point를 추가했을 때만 재빌드
source install/setup.bash
cd src/pinky_fleet && python3 -m pytest test
```

다른 워크스페이스(예: `~/pinky_pro`)와 동시에 source하지 않는다. 패키지 이름이 같아 어느 코드가 도는지 헷갈린다.

## 실행

- Gazebo(로봇 2대 + 관제): `ros2 launch pinky_fleet sim.launch.py` → [docs/sim.md](docs/sim.md)
- 실물: 로봇에서 `pinky_bringup`, PC에서 `ros2 launch pinky_fleet multi_robot.launch.py` → [docs/real.md](docs/real.md)

## 꼭 지킬 것

- 실물 로봇을 움직이는 명령(`cmd_vel` 발행, Nav2 목표)은 사람 확인 없이 실행하지 않는다.
- ⚠️ `pinky_bringup`은 `cmd_vel`이 끊겨도 마지막 속도로 계속 달린다. 조종을 끝내면 0 속도를 한 번 보낸다.
- 도메인은 실물 15/17, 시뮬 25/27. 시뮬은 이 PC 안에서만 통신하게 격리한다(`sim.launch.py`가 자동으로 한다).
- 경로·IP·도메인을 코드에 박지 않는다. launch 인자나 파라미터로 받는다. 토픽은 상대 이름(`cmd_vel`). `use_sim_time`은 launch에서 넘긴다.
- 커밋 금지: `build/` `install/` `log/`, 대용량(rosbag·영상·모델 가중치), 비밀번호·토큰.
- 변경은 작게, 한 커밋에 한 목적. 수정 후 해당 패키지를 빌드·테스트하고, 직접 실행 못 한 부분은 그렇다고 밝힌다.
- 설명과 커밋 메시지는 한국어, 코드 식별자는 영어.

## Git (요약)

`main`에 직접 push하지 않는다 → `<타입>/<짧은-설명>` 브랜치 → PR → 작성자가 **Squash and merge**.
커밋 제목: `<타입>(<범위>): <한국어 요약>` — 타입은 feat / fix / refactor / docs / test / chore. 상세: [docs/git.md](docs/git.md)
