#!/usr/bin/env python3

import asyncio
import websockets
import json
import time

async def simple_demo():
    """Simple demo to test phospho teleoperation"""
    
    websocket_url = "ws://localhost:8020/move/teleop/ws"
    
    print("🎮 Simple Phospho Demo")
    print(f"📡 Connecting to: {websocket_url}")
    
    try:
        async with websockets.connect(websocket_url) as websocket:
            print("✅ Connected to WebSocket server")
            
            # Send a simple test command
            test_command = {
                "x": 0.1,
                "y": 0.2,
                "z": 0.1,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 45.0,
                "open": 1,
                "source": "demo"
            }
            
            print(f"📤 Sending test command: {test_command}")
            await websocket.send(json.dumps(test_command))
            
            # Wait for response
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                response_data = json.loads(response)
                print(f"📥 Received response: {response_data}")
                print("✅ Demo successful!")
            except asyncio.TimeoutError:
                print("⏰ No response received (timeout)")
            except json.JSONDecodeError:
                print("⚠️ Invalid JSON response")
                
    except ConnectionRefusedError:
        print("❌ Connection refused. Start the test server first:")
        print("   python3 src/phospho_teleop/demo/test_server.py")
    except Exception as e:
        print(f"❌ Error: {e}")

def main():
    print("🎯 Simple Phospho Teleoperation Demo")
    print("=" * 40)
    
    try:
        asyncio.run(simple_demo())
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrupted by user")
    except Exception as e:
        print(f"❌ Demo failed: {e}")

if __name__ == "__main__":
    main() 