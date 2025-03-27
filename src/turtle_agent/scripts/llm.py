import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI

# Load environment variables from .env file
load_dotenv()

def get_llm(*args, **kwargs):
    """
    Create and return an OpenAI LLM instance for use with ROSA
    
    This function is designed to be flexible and accept various arguments
    
    Args:
        *args: Positional arguments
        **kwargs: Keyword arguments including potential 'streaming' parameter
    
    Returns:
        ChatOpenAI: Configured ChatOpenAI instance for use with ROSA
    """
    # Extract streaming parameter, default to False if not provided
    streaming = kwargs.get('streaming', False)
    verbose = kwargs.get('verbose', False)

    # Get the OpenAI API key from environment variables
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        raise ValueError("OPENAI_API_KEY not found in environment variables. Please set it in the .env file or system environment.")

    # Create OpenAI LLM wrapper using LangChain's ChatOpenAI
    openai_llm = ChatOpenAI(
        api_key=api_key,
        temperature=0,         # Deterministic output, adjust as needed
        model_name="gpt-4o-mini",  # You can change to "gpt-3.5-turbo" or another model if preferred
        streaming=streaming
    )

    if verbose:
        print(f"LLM initialized with streaming: {streaming}, model: gpt-4o-mini")

    return openai_llm

# Optional: Add error handling and logging
if __name__ == "__main__":
    try:
        llm = get_llm(streaming=False, verbose=True)
        print("LLM instance created successfully")
    except Exception as e:
        print(f"Error creating LLM instance: {e}")