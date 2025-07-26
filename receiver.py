import os
import cv2
import csv
import asyncio
from datetime import datetime
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole
from aiortc.contrib.signaling import TcpSocketSignaling

output_dir = "images"
os.makedirs(output_dir, exist_ok=True)
log_path = os.path.join(output_dir, "frame_log.csv")

# Start CSV log file
with open(log_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "Image"])

async def run():
    pc = RTCPeerConnection()
    signaling = TcpSocketSignaling("sender_ip_address", 1234)  # Change to your friend's IP
    await signaling.connect()

    offer = await signaling.receive()
    await pc.setRemoteDescription(offer)
    await pc.setLocalDescription(await pc.createAnswer())
    await signaling.send(pc.localDescription)

    @pc.on("track")
    def on_track(track):
        print("Receiving video track...")

        async def recv_video():
            index = 0
            while True:
                frame = await track.recv()
                img = frame.to_ndarray(format="bgr24")
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"{output_dir}/frame_{index}_{timestamp}.jpg"
                cv2.imwrite(filename, img)

                with open(log_path, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, filename])
                index += 1

        asyncio.ensure_future(recv_video())

    await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(run())
