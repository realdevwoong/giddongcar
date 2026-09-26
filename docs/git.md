# Git 컨벤션

## 브랜치

- `main`에 직접 push하지 않는다. 작업마다 브랜치를 만들고 PR로 합친다.
- 이름은 `<타입>/<짧은-설명>`. 예: `feat/camera-viewer`, `fix/nav2-footprint`, `docs/agents-md`
- 시작 전에 최신 상태를 받는다.

```bash
git switch main && git pull
git switch -c feat/camera-viewer
```

- "main 직접 push 금지"를 강제하려면 레포 오너가 GitHub Settings → Branches에서 보호 규칙을 켠다(필요한 승인 수 0이면 셀프 머지 가능).

## 커밋 메시지

```
<타입>(<범위>): <한국어 요약>

(필요하면 한 줄 띄우고 이유)
```

| 타입 | 언제 |
|---|---|
| `feat` | 기능 추가 |
| `fix` | 버그 수정 |
| `refactor` | 동작은 그대로, 구조만 개선 |
| `docs` | 문서 |
| `test` | 테스트 |
| `chore` | 설정, 빌드, 기타 |

- 범위(선택)는 바꾼 영역: `fleet`, `camera`, `nav`, `sim`, `bringup` 등.
- 요약은 50자 안팎, 무엇을 했는지. 한 커밋에 한 가지 변경. 빌드가 깨진 상태로 커밋하지 않는다.

```
feat(camera): 가제보 카메라 영상 구독 노드 추가
fix(nav): 좁은 통로에서 멈추는 footprint 값 수정
docs: AGENTS.md와 git 컨벤션 추가
```

## PR

- 제목은 커밋 메시지 규칙과 같게.
- 본문: **무엇을 왜 바꿨는지**, **테스트 방법**(가제보/실물, 실행 명령). 화면이 바뀌면 스크린샷.
- 작성자가 직접 머지한다(리뷰는 선택). **Squash and merge**, 머지한 브랜치는 삭제.
- 다른 사람이 만든 폴더를 옮기거나 지우는 PR은 머지 전에 당사자에게 한마디 한다.
