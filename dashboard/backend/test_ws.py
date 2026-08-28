import asyncio
import websockets
import json

async def test_ws():
    uri = "ws://localhost:8000/ws/telemetry"
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to WebSocket.")
            message = await websocket.recv()
            print(f"Received JSON: {message}")
            try:
                data = json.loads(message)
                if data.get("type") == "telemetry":
                    print("Test Passed: Received valid telemetry payload.")
                else:
                    print("Test Failed: JSON is not a telemetry payload.")
            except Exception as e:
                print(f"Test Failed: Could not parse JSON. {e}")
    except Exception as e:
        print(f"WebSocket connection failed: {e}")

if __name__ == "__main__":
    asyncio.run(test_ws())
