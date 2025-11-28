import warnings

# Suppress verbose warnings during test runs
warnings.filterwarnings("ignore", category=DeprecationWarning, module="langchain")
warnings.filterwarnings("ignore", message=".*ArbitraryTypeWarning.*", module="pydantic")
