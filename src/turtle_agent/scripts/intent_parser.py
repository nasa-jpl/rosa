from __future__ import annotations

from typing import Any, Dict

from llm import parse_intent_with_llm


class IntentParser:
    """사용자 자연어를 의도 구조체로 변환하는 파서입니다."""

    def __init__(self, llm: Any):
        """의도 파싱에 사용할 LLM 인스턴스를 초기화합니다."""
        self._llm = llm

    def parse(self, query: str) -> Dict[str, Any]:
        """입력 쿼리를 intent_norm 형식으로 파싱해 반환합니다."""
        return parse_intent_with_llm(self._llm, query)
