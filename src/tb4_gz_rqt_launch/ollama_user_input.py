#!/usr/bin/env python3
"""
Quick test script for Ollama + Phi-3 JSON extraction
Run: python3 ollama_test.py
"""

import json
import requests
import sys
import time

OLLAMA_API = "http://localhost:11434/api/generate"

SYSTEM_PROMPT = """You are a robot navigation parser. Your only job is to extract semantic targets from user text.

Output format: JSON only. No markdown, no conversational filler.

Keys: 
- 'target' (the destination object)
- 'relation' (spatial preposition like 'near', 'behind', 'left of')
- 'reference' (landmark object)

If no spatial constraint exists: set 'relation' and 'reference' to null.
Examples:
Input: 'Go to the chair' -> Output: {"target": "chair", "relation": null, "reference": null}
Input: 'Find the table near the window' -> Output: {"target": "table", "relation": "near", "reference": "window"}
Input: 'Move to the sofa left of the lamp' -> Output: {"target": "sofa", "relation": "left of", "reference": "lamp"}
Input: 'Go down the hall' -> Output: {"target": "hall", "relation": null, "reference": null}"""


def extract_json_from_response(response_text: str) -> dict:
    """Extract JSON from LLM response (handles markdown formatting and extra content)."""
    if '{' not in response_text or '}' not in response_text:
        return None
    
    # Find the first { and last }
    start = response_text.find('{')
    
    # Try to find the matching closing brace by counting braces
    brace_count = 0
    end = -1
    for i in range(start, len(response_text)):
        if response_text[i] == '{':
            brace_count += 1
        elif response_text[i] == '}':
            brace_count -= 1
            if brace_count == 0:
                end = i + 1
                break
    
    if end == -1:
        # Fallback: just use the last }
        end = response_text.rfind('}') + 1
    
    json_str = response_text[start:end].strip()
    
    try:
        return json.loads(json_str)
    except json.JSONDecodeError as e:
        # Last resort: remove markdown formatting
        json_str = json_str.replace('```json', '').replace('```', '').strip()
        try:
            return json.loads(json_str)
        except json.JSONDecodeError:
            return None

def parse_command(user_input: str) -> tuple[dict, float]:
    """
    Send user input to Phi-3 via Ollama and extract structured goal.
    
    Returns:
    tuple of (parsed_result, elapsed_time_seconds)
    """
    payload = {
        "model": "phi3",
        "prompt": f"Current Input: {user_input}",
        "system": SYSTEM_PROMPT,
        "stream": False
    }
    
    start_time = time.time()
        
    try:
        response = requests.post(OLLAMA_API, json=payload, timeout=120)
        response.raise_for_status()
        result = response.json()
        
        extracted = extract_json_from_response(result['response'])
        elapsed_time = time.time() - start_time
        
        return extracted, elapsed_time
        
    except requests.exceptions.RequestException as e:
        print(f"ERROR: Failed to connect to Ollama at {OLLAMA_API}")
        print(f"Details: {e}")
        return None, elapsed_time
    except Exception as e:
        print(f"ERROR: {e}")
        return None, elapsed_time

if __name__ == "__main__":  
    # Interactive user input test
    print("\n" + "=" * 60)
    print("[6] INTERACTIVE MODE TEST - Enter command")
    print("=" * 60)
    try:
        while True:
            user_input = input("\n Enter a navigation command (ctrl-c to end): ").strip()
            if user_input:
                print(f"\nInput: '{user_input}'")
                result, elapsed = parse_command(user_input)
                if result:
                    print(f"Output: {json.dumps(result, indent=2)}")
                    print(f"Processing time: {elapsed:.3f} seconds")
                else:
                    print(f" Failed to parse (took {elapsed:.3f}s)")
            else:
                print("Skipped interactive mode.")
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        pass
    except EOFError:
        print("\nNo input provided (EOF).")
    
    print("\n" + "=" * 60)
    print("Test complete!")
    print("=" * 60)
