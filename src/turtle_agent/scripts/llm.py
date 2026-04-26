#  Copyright (c) 2025. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import json
import os
from typing import Any, Dict


def get_llm(streaming: bool = False):
    """A helper function to get the LLM instance.

    Supports OpenAI (default), Anthropic and Ollama models.
    Set the LLM_PROVIDER env variable to switch between providers:
      - "openai" (default): uses OPENAI_API_KEY
      - "anthropic": uses ANTHROPIC_API_KEY
      - "ollama": uses local Ollama instance
    """
    import dotenv

    dotenv.load_dotenv(dotenv.find_dotenv())

    provider = os.getenv("LLM_PROVIDER", "openai").lower().strip()
    supported = ("openai", "anthropic", "ollama")
    if provider not in supported:
        raise ValueError(
            f"Unknown LLM_PROVIDER: '{provider}'. Must be one of: {', '.join(supported)}"
        )

    if provider == "openai":
        from langchain_openai import ChatOpenAI

        llm = ChatOpenAI(
            api_key=get_env_variable("OPENAI_API_KEY"),
            model=os.getenv("OPENAI_MODEL", "gpt-4o"),
            streaming=streaming,
        )
    elif provider == "anthropic":
        try:
            from langchain_anthropic import ChatAnthropic
        except ImportError:
            raise ImportError(
                "langchain-anthropic is required for Anthropic support. "
                "Install it with: pip install langchain-anthropic"
            )
        llm = ChatAnthropic(
            api_key=get_env_variable("ANTHROPIC_API_KEY"),
            model=os.getenv("ANTHROPIC_MODEL", "claude-sonnet-4-5"),
            streaming=streaming,
        )
    elif provider == "ollama":
        try:
            from langchain_ollama import ChatOllama
        except ImportError:
            raise ImportError(
                "langchain-ollama is required for Ollama support. "
                "Install it with: pip install langchain-ollama"
            )
        llm = ChatOllama(
            model=os.getenv("OLLAMA_MODEL", "llama3"),
            base_url=os.getenv("OLLAMA_BASE_URL", "http://localhost:11434"),
            streaming=streaming,
        )

    return llm


def parse_intent_with_llm(llm: Any, query: str) -> Dict[str, Any]:
    """
    Parse a user query into an intent_norm dict.

    Output format:
    {"task_family": "...", "slots": {...}}
    """
    prompt = (
        "You are an intent normalizer for turtlesim control.\n"
        "Return JSON only (no markdown).\n"
        "Schema:\n"
        '{ "task_family": "goto|trace_shape|rotate|move_object|unknown", "slots": { ... } }\n'
        "Rules:\n"
        "- task_family=trace_shape when user asks to draw/trace shapes.\n"
        "- for rectangle, set slots.shape='rectangle' and slots.size (float, default 2.0).\n"
        "- task_family=goto when user asks to move to coordinates; use slots.x, slots.y, optional slots.theta.\n"
        "- task_family=rotate when user asks turn/rotate; use slots.degrees and slots.direction(left/right).\n"
        "- task_family=move_object when user asks to move a box/object.\n"
        "- for move_object, prefer slots.object='box', slots.shape='rectangle', slots.size, "
        "slots.from_x, slots.from_y, slots.to_x, slots.to_y.\n"
        "- if unclear, return task_family='unknown' with empty slots.\n"
        f"User query: {query}"
    )
    try:
        response = llm.invoke(prompt)
        content = _message_to_text(response).strip()
        parsed = json.loads(content)
        return _normalize_intent(parsed)
    except Exception:
        return _fallback_intent_parse(query)


def _message_to_text(response: Any) -> str:
    content = getattr(response, "content", response)
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        chunks = []
        for block in content:
            if isinstance(block, dict) and "text" in block:
                chunks.append(str(block["text"]))
            elif hasattr(block, "text"):
                chunks.append(str(block.text))
            else:
                chunks.append(str(block))
        return "\n".join(chunks)
    return str(content)


def _normalize_intent(raw: Dict[str, Any]) -> Dict[str, Any]:
    task_family = str(raw.get("task_family", "unknown")).strip().lower()
    slots = raw.get("slots", {})
    if not isinstance(slots, dict):
        slots = {}
    if task_family not in {"goto", "trace_shape", "rotate", "move_object"}:
        task_family = "unknown"
        slots = {}
    return {"task_family": task_family, "slots": slots}


def _fallback_intent_parse(query: str) -> Dict[str, Any]:
    text = query.lower()
    if any(token in text for token in ("박스", "box")) and any(
        token in text for token in ("옮", "이동", "move")
    ):
        return {
            "task_family": "move_object",
            "slots": {
                "object": "box",
                "shape": "rectangle",
                "size": 2.0,
                "from_x": 2.0,
                "from_y": 2.0,
                "to_x": 7.0,
                "to_y": 7.0,
            },
        }
    if any(token in text for token in ("rectangle", "사각", "네모")):
        return {"task_family": "trace_shape", "slots": {"shape": "rectangle", "size": 2.0}}
    if any(token in text for token in ("rotate", "turn", "회전")):
        return {"task_family": "rotate", "slots": {"degrees": 90.0, "direction": "left"}}
    return {"task_family": "unknown", "slots": {}}


def get_env_variable(var_name: str) -> str:
    """
    Retrieves the value of the specified environment variable.

    Args:
        var_name (str): The name of the environment variable to retrieve.

    Returns:
        str: The value of the environment variable.

    Raises:
        ValueError: If the environment variable is not set.

    This function provides a consistent and safe way to retrieve environment variables.
    By using this function, we ensure that all required environment variables are present
    before proceeding with any operations. If a variable is not set, the function will
    raise a ValueError, making it easier to debug configuration issues.
    """
    value = os.getenv(var_name)
    if value is None:
        msg = f"Environment variable {var_name} is not set."
        raise ValueError(msg)
    return value
