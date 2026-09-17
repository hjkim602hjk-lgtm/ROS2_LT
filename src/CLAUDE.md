@AGENTS.md

## Claude Code 전용

- 위 AGENTS.md가 이 프로젝트 규칙의 원본이다. 내용 수정은 AGENTS.md에서 한다 (Codex와 공유).
- 허용된 명령 목록은 `.claude/settings.local.json`에 있다. 권한 프롬프트가 반복되면 거기에 추가한다.
- 장시간 실행되는 노드(`ros2 launch`, `ros2 run`)는 `run_in_background: true`로 띄우고, 확인이 끝나면 반드시 종료시킨다.
