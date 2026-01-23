#!/usr/bin/env python3
"""
Quick test script for Ollama + Phi-3 JSON extraction
Run: python3 ollama_test.py
"""

import json
import requests
import sys

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
Input: 'Move to the sofa left of the lamp' -> Output: {"target": "sofa", "relation": "left of", "reference": "lamp"}"""


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


def parse_command(user_input: str) -> dict:
    """
    Send user input to Phi-3 via Ollama and extract structured goal.
    
    Returns:
    {
        "target": str (destination object),
        "relation": str | null (spatial relation),
        "reference": str | null (landmark)
    }
    """
    payload = {
        "model": "phi3",
        "prompt": f"Current Input: {user_input}",
        "system": SYSTEM_PROMPT,
        "stream": False
    }
    
    try:
        response = requests.post(OLLAMA_API, json=payload, timeout=120)
        response.raise_for_status()
        result = response.json()
        
        extracted = extract_json_from_response(result['response'])
        return extracted
        
    except requests.exceptions.RequestException as e:
        print(f"ERROR: Failed to connect to Ollama at {OLLAMA_API}")
        print(f"Details: {e}")
        print("Make sure Ollama is running: ollama serve")
        return None
    except Exception as e:
        print(f"ERROR: {e}")
        return None


if __name__ == "__main__":
    test_cases = [
        "Go to the chair",
        "Find the table near the window",
        "Navigate to the kitchen",
        "Move to the sofa left of the lamp",
        "Go to the moon",
    ]
    
    print("=" * 60)
    print("OLLAMA + PHI-3 COMMAND PARSER TEST")
    print("=" * 60)
    
    for i, cmd in enumerate(test_cases, 1):
        print(f"\n[{i}] Input: '{cmd}'")
        result = parse_command(cmd)
        if result:
            print(f"    Output: {json.dumps(result, indent=6)}")
        else:
            print(f"    ❌ Failed to parse")
    
    # Interactive user input test
    print("\n" + "=" * 60)
    print("[6] INTERACTIVE MODE - Enter your own command")
    print("=" * 60)
    try:
        user_input = input("\n🤖 Enter a navigation command (or press Enter to skip): ").strip()
        if user_input:
            print(f"\nInput: '{user_input}'")
            result = parse_command(user_input)
            if result:
                print(f"Output: {json.dumps(result, indent=2)}")
            else:
                print(f"❌ Failed to parse")
        else:
            print("Skipped interactive mode.")
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except EOFError:
        print("\nNo input provided (EOF).")
    
    print("\n" + "=" * 60)
    print("✅ Test complete!")
    print("=" * 60)
