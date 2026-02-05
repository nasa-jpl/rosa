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

import os

import dotenv
from langchain_openai import ChatOpenAI


def get_llm(streaming: bool = False):
    """A helper function to get the LLM instance.

    Supports OpenAI (default), Anthropic and Ollama models.
    Set the LLM_PROVIDER env variable to switch between providers:
      - "openai" (default): uses OPENAI_API_KEY
      - "anthropic": uses ANTHROPIC_API_KEY
      - "ollama": uses local Ollama instance
    """
    dotenv.load_dotenv(dotenv.find_dotenv())

    provider = os.getenv("LLM_PROVIDER", "openai").lower().strip()
    supported = ("openai", "anthropic", "ollama")
    if provider not in supported:
        raise ValueError(
            f"Unknown LLM_PROVIDER: '{provider}'. Must be one of: {', '.join(supported)}"
        )

    if provider == "openai":
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
