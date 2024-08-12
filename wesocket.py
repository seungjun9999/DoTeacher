import asyncio
import websockets

async def listen(websocket):
    async for message in websocket:
        print(f"Received message from server: {message}")

async def send_messages(websocket):
    # init
    message = "112"
    await websocket.send(message)
    print(f"Sent message to server: {message}")

async def connect():
    uri = "ws://localhost:8080/ws"
    # uri = "ws://i11d102.p.ssafy.io:8081/ws"
    async with websockets.connect(uri) as websocket:
        listen_task = asyncio.create_task(listen(websocket))
        send_task = asyncio.create_task(send_messages(websocket))
        await asyncio.gather(listen_task, send_task)

asyncio.run(connect())