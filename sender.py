import cv2
from aiortc import MediaStreamTrack, RTCPeerConnection
from aiortc.contrib.signaling import TcpSocketSignaling
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
import asyncio

class CameraStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        ret, frame = self.cap.read()
        if not ret:
            return None
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        new_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        return new_frame

async def run():
    signaling = TcpSocketSignaling("0.0.0.0", 8765)
    await signaling.connect()
    pc = RTCPeerConnection()
    pc.addTrack(CameraStreamTrack())

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)
    await signaling.send(pc.localDescription)

    answer = await signaling.receive()
    await pc.setRemoteDescription(answer)

    print("🟢 Streaming started. Press Ctrl+C to stop.")
    await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(run())
