# Memory Schema Contract

팀 협의 우선순위(#38, #48, #52)를 기준으로 현재 메모리 스키마 계약을 명시합니다.

## #38 Short-term execution trace

- 경로: `short_term[*].evidence.execution_trace.steps[*]`
- 필수 필드:
  - `t_ms`
  - `skill`
  - `args`
  - `status`
  - `result`
  - `source`
  - `start_pose: {x, y, theta}`
  - `final_pose: {x, y, theta}`

레거시 호환을 위해 `evidence.execution_steps`도 유지합니다.

## #48 Memory module ownership

- `memory_io.py`: 입출력/경로/JSONL 정규화
- `memory_short_term.py`: short-term 생성/업데이트/최종화
- `memory_long_term.py`: long-term 압축/증거 집계/교훈 생성

## #52 control-worker schema

`memory_modules.schema_contracts`에 다음 타입이 정의됩니다.

- `WorkerTask`
  - `task_id`
  - `prompt`
  - `timeout`
  - `priority`
- `WorkerResult`
  - `task_id`
  - `status`
  - `result`
  - `error`
