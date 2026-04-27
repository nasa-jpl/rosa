# Task #16: 거북이 로그 → 기억 스키마 변환 파이프라인

## 목표

`location.jsonl`(포즈 로그)과 `command.jsonl`(스킬/의도 로그)을 읽어, 정의된 단기/장기 기억 스키마 포맷으로 변환해 저장하는 파이프라인을 구현한다.

---

## 입력 파일

### `logs/<date>/<session>/<turtle>/location.jsonl`
PoseHub(#17)가 생성. 레코드 예시:
```json
{"t_ros": {"secs": 1777202848, "nsecs": 956991000}, "x": 5.54, "y": 5.54, "theta": 0.0, "linear_velocity": 0.0, "angular_velocity": 0.0}
```

### `logs/<date>/<session>/<turtle>/command.jsonl`
#15가 생성. 레코드 타입이 `type` 필드로 구분됨:
```json
{"type": "intent", "t_ms": 1234567890, "task_family": "trace_shape", "slots": {"shape": "rectangle", "size": 2.0}}
{"type": "skill", "t_ms": 1234567891, "skill": "move_forward", "args": {"distance": 2.0}, "status": "success", "result": "moved 2.0m"}
```

> **주의**: 경로와 파일명을 하드코딩하지 말고 어댑터 함수로 분리할 것. `session_id`, `turtle_id` 추출도 경로 파싱에만 의존하지 않도록 설계할 것.

---

## 출력 파일

저장 위치: `src/turtle_agent/scripts/memory/`

### 단기 기억: `memory/short_term/<session_id>.jsonl`

움직임이 끝날 때마다 한 줄 추가(append).

스키마:
```json
{
  "session_id": "uuid",
  "turtle_id": "turtle1",
  "clock": {
    "unix_ms": 1234567890,
    "frame_id": "world"
  },
  "active_goal": {
    "natural_language": "박스를 옮겨",
    "constraints": {}
  },
  "plan": {
    "steps": [
      {"name": "rotate", "status": "success"},
      {"name": "forward", "status": "success"}
    ],
    "current_step_idx": 2
  },
  "execution_trace": {
    "steps": [
      {
        "t_ms": 1234567890,
        "pose": {"x": 5.54, "y": 5.54, "theta": 0.0},
        "skill_invocations": [
          {"skill": "move_forward", "args": {"distance": 2.0}, "status": "success", "result": "moved 2.0m"}
        ],
        "events": []
      }
    ]
  }
}
```

### 장기 기억: `memory/long_term.jsonl`

단기 기억 **5회 누적** 또는 **세션 종료** 시 변환해 한 줄 추가(append).

스키마:
```json
{
  "record_id": "uuid",
  "record_type": "raw_episode",
  "turtle_id": "turtle1",
  "payload": {
    "operation": {
      "nl_goal": {"text": "박스를 옮겨"},
      "intent_norm": {"task_family": "trace_shape", "slots": {"shape": "rectangle"}}
    },
    "action_trace": [
      {"t_ms": 1234567890, "skill": "move_forward", "args": {"distance": 2.0}, "status": "success", "result": "moved 2.0m"}
    ],
    "outcome": {
      "success": true,
      "terminal_reason": "goal_reached"
    }
  }
}
```

---

## 저장 트리거

| 기억 종류 | 트리거 |
|---|---|
| 단기 기억 | 움직임(스킬 시퀀스) 1회 완료 시 |
| 장기 기억 | 단기 기억 5회 누적 시 OR 세션 종료 시 |

---

## 설계 주의사항

- `location.jsonl` 경로 구조는 #17(PoseHub)에 의해 이미 `logs/<date>/<session>/<turtle>/location.jsonl`로 확정됨.
- 같은 turtle name을 kill 후 재사용하면 같은 `location.jsonl`에 이어 기록됨. 생명주기 인스턴스별 분리는 이번 범위 밖.
- JSONL 사용 이유: 매 움직임마다 append 방식으로 기록하기 위함.

---

## 대상 파일

- `src/turtle_agent/scripts/pose_logger.py` — 입력 레퍼런스
- 신규: `src/turtle_agent/scripts/memory_converter.py` — 변환 로직
- 신규: `src/turtle_agent/scripts/memory/` — 출력 디렉터리

## 연관 이슈

- **#15** `command.jsonl` 생성 담당
- **#17** `location.jsonl` 생성 담당 (이미 완료)
