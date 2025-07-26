import cv2
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.signaling import TcpSocketSignaling
from aiortc.contrib.media import MediaRelay
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
        return VideoFrame.from_ndarray(frame, format="rgb24")

async def run():
    pc = RTCPeerConnection()
    signaling = TcpSocketSignaling("0.0.0.0", 1234)
    await signaling.connect()

    track = CameraStreamTrack()
    pc.addTrack(track)

    await signaling.send(pc.createOffer())
    await pc.setLocalDescription(await pc.createOffer())
    await signaling.send(pc.localDescription)

    answer = await signaling.receive()
    await pc.setRemoteDescription(answer)

    await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(run())
