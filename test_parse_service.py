#!/usr/bin/env python3
"""
Interactive test script for ROS 2 Command Parser Service
Tests the /parse_command service with user input

Usage:
    python3 test_parse_service.py

Prerequisites:
    - Ollama running: ollama serve
    - ROS packages built: colcon build
    - Environment sourced: source install/setup.bash
"""

import subprocess
import time
import sys

def main():
    print("=" * 70)
    print("ROS 2 Command Parser Service - Interactive Test")
    print("=" * 70)
    print("\n[1/3] Starting parse_command_node...")

    # Start the node
    node_proc = subprocess.Popen(
        ["bash", "-c", 
         "source install/setup.bash && source /opt/ros/jazzy/setup.bash && parse_command_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd="/home/maxrec/ros2_jazzy/vision-language-navigator"
    )

    # Wait for node to start
    time.sleep(3)
    print("[✓] parse_command_node started on /parse_command\n")

    # Show instructions
    print("[2/3] Ready for commands!")
    print("-" * 70)
    print("Enter natural language navigation commands:")
    print("  Examples:")
    print("    • 'go to the chair'")
    print("    • 'move to the table near the window'")
    print("    • 'navigate to the kitchen'")
    print("  Commands: 'quit' or 'exit' to stop\n")

    try:
        while True:
            # Get user input
            user_command = input("🤖 Enter command: ").strip()
            
            # Check for exit
            if user_command.lower() in ['quit', 'exit', 'q']:
                print("\n[✓] Exiting...")
                break
            
            # Skip empty input
            if not user_command:
                print("  ⚠ Empty input, try again\n")
                continue
            
            print(f"\n  [Processing] '{user_command}'...\n")
            
            # Call the service
            cmd = f"""source install/setup.bash && source /opt/ros/jazzy/setup.bash && \
ros2 service call /parse_command tb4_interfaces/srv/ParseCommand '{{command_text: "{user_command}"}}' 2>/dev/null"""
            
            result = subprocess.run(
                ["bash", "-c", cmd],
                capture_output=True,
                text=True,
                timeout=130,
                cwd="/home/maxrec/ros2_jazzy/vision-language-navigator"
            )
            
            if result.returncode == 0:
                # Parse and display output
                lines = result.stdout.strip().split('\n')
                response_started = False
                for line in lines:
                    if 'response:' in line:
                        response_started = True
                        print("  Response:")
                    elif response_started and line.strip():
                        # Clean up formatting
                        clean_line = line.strip()
                        if clean_line.startswith('tb4_interfaces'):
                            # Extract just the values
                            for field in ['target_object=', 'relation_label=', 'reference_object=', 'success=']:
                                if field in clean_line:
                                    parts = clean_line.split(', ')
                                    for part in parts:
                                        if field in part:
                                            print(f"    {part}")
                        else:
                            print(f"    {clean_line}")
                print()
            else:
                print(f"  ❌ Service call failed\n")

    except KeyboardInterrupt:
        print("\n\n[✓] Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        # Cleanup
        print("\n[3/3] Cleaning up...")
        print("  Stopping parse_command_node...")
        node_proc.terminate()
        try:
            node_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            node_proc.kill()
        
        print("\n" + "=" * 70)
        print("✅ Test complete!")
        print("=" * 70)

if __name__ == "__main__":
    main()
