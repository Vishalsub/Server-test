# Isaac Script (run inside Isaac Sim)
from pxr import UsdGeom
import omni.usd as usd
import rclpy, yaml, os
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SizePub(Node):
    def __init__(self, prim_path="/World/obj_1", yaml_path="config/scene.yaml"):
        super().__init__("obj1_size_pub")
        self._stage = usd.get_context().get_stage()
        self._prim = self._stage.GetPrimAtPath(prim_path)
        self._bbox = UsdGeom.BBoxCache(0, ["default"])
        self.pub = self.create_publisher(Float32MultiArray, "/obj_1/size_xyz", 10)
        self.yaml_path = yaml_path
        self.timer = self.create_timer(0.5, self.tick)

    def _write_yaml(self, sx, sy, sz):
        # Load existing YAML (if any), then update obj_1.size_xyz
        data = {}
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, "r") as f:
                try:
                    data = yaml.safe_load(f) or {}
                except Exception:
                    data = {}
        data.setdefault("scene", {}).setdefault("objects", {}).setdefault("obj_1", {})
        data["scene"]["objects"]["obj_1"]["size_xyz"] = [float(sx), float(sy), float(sz)]
        with open(self.yaml_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def tick(self):
        bound = self._bbox.ComputeWorldBound(self._prim)
        size = bound.GetBox().GetSize()  # meters
        sx, sy, sz = float(size[0]), float(size[1]), float(size[2])

        # Publish to ROS
        msg = Float32MultiArray(); msg.data = [sx, sy, sz]
        self.pub.publish(msg)

        # Persist to YAML (optional but handy)
        self._write_yaml(sx, sy, sz)

rclpy.init()
node = SizePub(prim_path="/World/obj_1", yaml_path="config/scene.yaml")
rclpy.spin(node)
