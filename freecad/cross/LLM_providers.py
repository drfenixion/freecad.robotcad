"""LLM Provider abstraction layer for RobotCAD.

Supports OpenRouter, Ollama, and RouterAI providers for LLM API calls.
"""

from __future__ import annotations

import time
from typing import Optional, Callable

import FreeCAD as fc

try:
    import requests
except ImportError:
    requests = None

from .wb_utils import get_workbench_param, set_workbench_param

# Preference keys for LLM settings
PREF_LLM_PROVIDER = 'llm_provider'
PREF_OPENROUTER_API_KEY = 'openrouter_api_key'
PREF_OPENROUTER_MODEL = 'openrouter_model'
PREF_OLLAMA_MODEL = 'ollama_model'
PREF_ROUTERAI_API_KEY = 'routerai_api_key'
PREF_ROUTERAI_MODEL = 'routerai_model'

# Default values
DEFAULT_OPENROUTER_MODEL = 'google/gemini-3-flash-preview'
DEFAULT_OLLAMA_MODEL = 'gemini-3-flash-preview:cloud'
DEFAULT_ROUTERAI_MODEL = 'google/gemini-3-flash-preview'

# Provider names as displayed in UI
PROVIDER_OPENROUTER = 'OpenRouter'
PROVIDER_OLLAMA = 'Ollama'
PROVIDER_ROUTERAI = 'RouterAI'

AVAILABLE_PROVIDERS = [PROVIDER_OPENROUTER, PROVIDER_OLLAMA, PROVIDER_ROUTERAI]


def get_current_provider() -> str:
    """Get the currently selected LLM provider."""
    return get_workbench_param(PREF_LLM_PROVIDER, PROVIDER_OPENROUTER)


def set_current_provider(provider: str) -> None:
    """Set the current LLM provider."""
    if provider in AVAILABLE_PROVIDERS:
        set_workbench_param(PREF_LLM_PROVIDER, provider)


def get_openrouter_api_key() -> str:
    """Get the OpenRouter API key."""
    return get_workbench_param(PREF_OPENROUTER_API_KEY, '')


def set_openrouter_api_key(key: str) -> None:
    """Set the OpenRouter API key."""
    set_workbench_param(PREF_OPENROUTER_API_KEY, key)


def get_openrouter_model() -> str:
    """Get the OpenRouter model name."""
    return get_workbench_param(PREF_OPENROUTER_MODEL, DEFAULT_OPENROUTER_MODEL)


def set_openrouter_model(model: str) -> None:
    """Set the OpenRouter model name."""
    set_workbench_param(PREF_OPENROUTER_MODEL, model)


def get_ollama_model() -> str:
    """Get the Ollama model name."""
    return get_workbench_param(PREF_OLLAMA_MODEL, DEFAULT_OLLAMA_MODEL)


def set_ollama_model(model: str) -> None:
    """Set the Ollama model name."""
    set_workbench_param(PREF_OLLAMA_MODEL, model)


def get_routerai_api_key() -> str:
    """Get the RouterAI API key."""
    return get_workbench_param(PREF_ROUTERAI_API_KEY, '')


def set_routerai_api_key(key: str) -> None:
    """Set the RouterAI API key."""
    set_workbench_param(PREF_ROUTERAI_API_KEY, key)


def get_routerai_model() -> str:
    """Get the RouterAI model name."""
    return get_workbench_param(PREF_ROUTERAI_MODEL, DEFAULT_ROUTERAI_MODEL)


def set_routerai_model(model: str) -> None:
    """Set the RouterAI model name."""
    set_workbench_param(PREF_ROUTERAI_MODEL, model)


def call_openrouter_api(system_prompt: str, user_prompt: str, log_callback: Optional[Callable] = None) -> str:
    """Call LLM via OpenRouter API.
    
    Args:
        system_prompt: System instruction for the LLM.
        user_prompt: User's prompt.
        log_callback: Optional callback for logging progress.
    
    Returns:
        Generated content string.
    
    Raises:
        ValueError: If API key is not set.
        RuntimeError: If API call fails.
    """
    api_key = get_openrouter_api_key()
    model = get_openrouter_model()
    
    if not api_key:
        raise ValueError("OpenRouter API key not set. Please configure it in Workbench settings.")
    
    if log_callback:
        log_callback(f"Using OpenRouter with model: {model}")
    
    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        "temperature": 0.2,
        "max_tokens": 8192,
        "provider": {
            "allow_fallbacks": False,
            "order": [],
            "ignore": []
        },
        "cache": {
            "control": {"no_store": True}
        }
    }
    
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
        "HTTP-Referer": "",
        "X-Title": "RobotCAD",
        "Cache-Control": "no-cache, no-store, must-revalidate",
        "Pragma": "no-cache",
        "Expires": "0"
    }
    
    if log_callback:
        log_callback("Sending request to OpenRouter API...")
    
    try:
        response = requests.post(
            "https://openrouter.ai/api/v1/chat/completions",
            json=payload,
            headers=headers,
            timeout=300
        )
        # import debugpy; debugpy.breakpoint()
        if response.status_code == 200:
            result = response.json()
            # Check if response has expected structure
            if "choices" not in result:
                # Check if it's an error response
                error_info = result.get("error", {})
                error_message = error_info.get("message", "Unknown error")
                error_code = error_info.get("code", "unknown")
                
                # Provider error (code 524 = provider timeout/error)
                if error_code == 524:
                    error_detail = f"Provider error: {error_message} (code {error_code}). The model provider timed out or returned an error."
                else:
                    error_detail = f"API error: {error_message} (code {error_code})"
                
                if log_callback:
                    log_callback(f"ERROR: {error_detail}")
                raise RuntimeError(error_detail)
            if not result["choices"]:
                raise RuntimeError("OpenRouter API returned empty choices array.")
            generated_content = result["choices"][0]["message"]["content"]
            if not generated_content:
                raise RuntimeError("OpenRouter API returned empty content.")
            if log_callback:
                log_callback("Successfully received response from OpenRouter API")
            return generated_content
        else:
            raise RuntimeError(
                f"OpenRouter API request failed with status {response.status_code}: {response.text}"
            )
    
    except requests.exceptions.RequestException as e:
        raise RuntimeError(f"Network error when calling OpenRouter API: {str(e)}")


def call_ollama_api(system_prompt: str, user_prompt: str, log_callback: Optional[Callable] = None) -> str:
    """Call LLM via Ollama API.
    
    Args:
        system_prompt: System instruction for the LLM.
        user_prompt: User's prompt.
        log_callback: Optional callback for logging progress.
    
    Returns:
        Generated content string.
    
    Raises:
        RuntimeError: If API call fails.
    """
    model = get_ollama_model()
    
    if log_callback:
        log_callback(f"Using Ollama with model: {model}")
    
    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        "stream": False,
        "options": {
            "temperature": 0.2,
            "num_predict": 8192
        }
    }
    
    if log_callback:
        log_callback("Sending request to Ollama API (localhost:11434)...")
    
    try:
        response = requests.post(
            "http://localhost:11434/api/chat",
            json=payload,
            timeout=600
        )
        
        if response.status_code == 200:
            result = response.json()
            generated_content = result.get("message", {}).get("content", "")
            if not generated_content:
                raise RuntimeError("Ollama API returned empty content.")
            if log_callback:
                log_callback("Successfully received response from Ollama API")
            return generated_content
        else:
            raise RuntimeError(
                f"Ollama API request failed with status {response.status_code}: {response.text}"
            )
    
    except requests.exceptions.RequestException as e:
        raise RuntimeError(f"Network error when calling Ollama API: {str(e)}")


def call_routerai_api(system_prompt: str, user_prompt: str, log_callback: Optional[Callable] = None) -> str:
    """Call LLM via RouterAI API.
    
    Uses OpenAI-compatible API endpoint via requests (no openai package required).
    
    Args:
        system_prompt: System instruction for the LLM.
        user_prompt: User's prompt.
        log_callback: Optional callback for logging progress.
    
    Returns:
        Generated content string.
    
    Raises:
        ValueError: If API key is not set.
        RuntimeError: If API call fails.
    """
    api_key = get_routerai_api_key()
    model = get_routerai_model()
    
    if not api_key:
        raise ValueError("RouterAI API key not set. Please configure it in Workbench settings.")
    
    if log_callback:
        log_callback(f"Using RouterAI with model: {model}")
    
    payload = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        "temperature": 0.2,
        "max_tokens": 8192
    }
    
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }
    
    if log_callback:
        log_callback("Sending request to RouterAI API...")
    
    try:
        response = requests.post(
            "https://routerai.ru/api/v1/chat/completions",
            json=payload,
            headers=headers,
            timeout=300
        )
        
        if response.status_code == 200:
            result = response.json()
            generated_content = result["choices"][0]["message"]["content"]
            if not generated_content:
                raise RuntimeError("RouterAI API returned empty content.")
            if log_callback:
                log_callback("Successfully received response from RouterAI API")
            return generated_content
        else:
            raise RuntimeError(
                f"RouterAI API request failed with status {response.status_code}: {response.text}"
            )
    
    except requests.exceptions.RequestException as e:
        raise RuntimeError(f"Network error when calling RouterAI API: {str(e)}")


def call_llm_provider(system_prompt: str, user_prompt: str, log_callback: Optional[Callable] = None) -> str:
    """Call LLM using the currently configured provider.
    
    Args:
        system_prompt: System instruction for the LLM.
        user_prompt: User's prompt.
        log_callback: Optional callback for logging progress.
    
    Returns:
        Generated content string.
    
    Raises:
        RuntimeError: If generation fails.
    """
    provider = get_current_provider()
    
    if log_callback:
        log_callback(f"Selected provider: {provider}")
    
    # disable cache
    system_prompt = system_prompt + '\n timestamp: ' + str(time.time())

    if provider == PROVIDER_OPENROUTER:
        return call_openrouter_api(system_prompt, user_prompt, log_callback)
    elif provider == PROVIDER_OLLAMA:
        return call_ollama_api(system_prompt, user_prompt, log_callback)
    elif provider == PROVIDER_ROUTERAI:
        return call_routerai_api(system_prompt, user_prompt, log_callback)
    else:
        raise RuntimeError(f"Unknown LLM provider: {provider}")
