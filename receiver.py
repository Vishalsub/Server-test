import os
import cv2
import csv
import asyncio
from datetime import datetime
from aiortc import RTCPeerConnection
from aiortc.contrib.signaling import TcpSocketSignaling

SAVE_DIR = "images"
os.makedirs(SAVE_DIR, exist_ok=True)
LOG_PATH = os.path.join(SAVE_DIR, "frame_log.csv")

with open(LOG_PATH, "w", newline="") as f:
    csv.writer(f).writerow(["Timestamp", "Image"])

async def run():
    signaling = TcpSocketSignaling("SENDER_IP_ADDRESS", 8765)  # 👈 Replace this
    await signaling.connect()
    pc = RTCPeerConnection()

    offer = await signaling.receive()
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    await signaling.send(pc.localDescription)

    @pc.on("track")
    def on_track(track):
        print("📷 Receiving video feed...")

        async def grab_frames():
            index = 0
            while True:
                frame = await track.recv()
                img = frame.to_ndarray(format="bgr24")
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"{SAVE_DIR}/frame_{index}_{timestamp}.jpg"
                cv2.imwrite(filename, img)

                with open(LOG_PATH, "a", newline="") as f:
                    csv.writer(f).writerow([timestamp, filename])
                index += 1

        asyncio.ensure_future(grab_frames())

    await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(run())
