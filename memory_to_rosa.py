#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from typing import Any, Iterable, List


def _stringify(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, str):
        return value.strip()
    if isinstance(value, (int, float, bool)):
        return str(value)
    return json.dumps(value, ensure_ascii=False)


def _iter_values(value: Any) -> Iterable[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, dict):
        return [value]
    if isinstance(value, str):
        text = value.strip()
        return [text] if text else []
    return [value]


def _pick_position_text(incident: Any) -> str:
    if not isinstance(incident, dict):
        return _stringify(incident)

    for key in ("position", "location", "pos", "point", "coordinates"):
        if key in incident:
            pos = incident[key]
            if isinstance(pos, dict):
                x = pos.get("x")
                y = pos.get("y")
                if x is not None and y is not None:
                    return f"x={x}, y={y}"
            return _stringify(pos)

    x = incident.get("x")
    y = incident.get("y")
    if x is not None and y is not None:
        return f"x={x}, y={y}"

    return _stringify(incident)


def _robot_prefix(payload: dict[str, Any], path: Path) -> str:
    persona_id = _stringify(payload.get("persona_id"))
    if persona_id:
        return persona_id
    persona_name = _stringify(payload.get("persona"))
    if persona_name:
        return persona_name
    return path.stem


def build_commands(persona_name: str, payload: dict[str, Any]) -> List[str]:
    commands: List[str] = []

    long_term = payload.get("long_term_memory") or {}
    short_term = payload.get("short_term_memory") or {}

    for rule in _iter_values(long_term.get("learned_rules")):
        rule_text = _stringify(rule)
        if rule_text:
            commands.append(
                f"[{persona_name}] note_preference 툴을 호출해줘: {rule_text}"
            )

    for incident in _iter_values(long_term.get("past_incidents")):
        pos_text = _pick_position_text(incident)
        if pos_text:
            commands.append(
                f"[{persona_name}] avoid_position 툴을 호출해줘: {pos_text}"
            )

    task_text = _stringify(short_term.get("current_task"))
    if not task_text:
        active_goal = payload.get("active_goal", {})
        if isinstance(active_goal, dict):
            task_text = _stringify(active_goal.get("natural_language"))
    if task_text:
        commands.append(f"[{persona_name}] {task_text}")

    return commands


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} JSON 최상위는 object여야 합니다.")
    return data


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "persona 메모리를 ROSA 터미널용 자연어 명령어로 변환합니다."
        )
    )
    parser.add_argument(
        "persona_files",
        nargs="*",
        default=[
            "outputs/persona_A.json",
            "outputs/persona_B.json",
            "outputs/persona_C.json",
        ],
        help="입력 persona JSON 파일 경로들",
    )
    parser.add_argument(
        "--robot",
        action="append",
        default=[],
        help=(
            "특정 로봇만 출력 (예: --robot ROBOT_C). "
            "여러 번 지정 가능."
        ),
    )
    args = parser.parse_args()

    all_commands: List[str] = []
    robot_filters = {item.strip().upper() for item in args.robot if item.strip()}

    for file_arg in args.persona_files:
        path = Path(file_arg)
        if not path.exists():
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {path}")

        payload = read_json(path)
        persona_name = _robot_prefix(payload, path)
        if robot_filters and persona_name.upper() not in robot_filters:
            continue
        all_commands.extend(build_commands(str(persona_name), payload))

    for idx, cmd in enumerate(all_commands, start=1):
        print(f"{idx:02d}. {cmd}")


if __name__ == "__main__":
    main()
