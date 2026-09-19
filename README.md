# giddongcar

Pinky Pro 다중 로봇 관제 프로젝트 (ROS2 workspace: `pinky_pro/`).

## 워크스페이스 설정 (팀원 최초 1회)

**반드시 홈 디렉터리 바로 아래(`~/giddongcar`)에 클론하세요.** 다른 경로에 클론하면 로컬 빌드 캐시(`pinky_pro/build`, `pinky_pro/install`)가 그 경로를 기억해서, 나중에 폴더를 옮기면 빌드가 깨집니다.

```bash
cd ~
git clone git@github.com:realdevwoong/giddongcar.git
cd giddongcar
source /opt/ros/jazzy/setup.bash   # 사용 중인 ROS2 배포판에 맞게
./setup.sh
```

`setup.sh`가 `rosdep install`과 `colcon build`를 한 번에 해줍니다.

이후 매 터미널을 열 때마다:
```bash
source ~/giddongcar/pinky_pro/install/setup.bash
```

## 주의

- 클론한 폴더(`~/giddongcar`)를 나중에 다른 위치로 옮기지 마세요.
- 옮겨야 한다면 빌드 캐시를 지우고 다시 빌드하세요:
  ```bash
  rm -rf pinky_pro/build pinky_pro/install pinky_pro/log
  ./setup.sh
  ```
