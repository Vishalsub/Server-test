# save as: scene_sync_cube.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive

CUBE_FRAME   = "_09_gelatin_box"   # <-- set to your Isaac child frame
WORLD        = "world"
OBJECT_NAME  = "isaac_box"
BOX_SIZE     = (0.20, 0.15, 0.10)  # meters (x, y, z)

FLOOR_NAME   = "floor"
FLOOR_SIZE   = (5.0, 5.0, 0.02)    # big slab as ground
FLOOR_Z      = -0.01               # top flush at z=0

class SceneSync(Node):
    def __init__(self):
        super().__init__("scene_sync_cube")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.scene = PlanningSceneInterface(synchronous=True)

        # one-shot timer to add ground after PlanningSceneMonitor is up
        self._floor_added = False
        self.create_timer(1.0, self._add_floor_once)

        # periodic update of Isaac cube into planning scene
        self.create_timer(0.1, self._tick)  # 10 Hz

    def _add_floor_once(self):
        if self._floor_added:
            return
        # if PlanningScene not ready yet, names may be empty; retry next tick
        names = set(self.scene.get_known_object_names())
        if FLOOR_NAME not in names:
            pose = PoseStamped()
            pose.header.frame_id = WORLD
            pose.pose.position.z = FLOOR_Z
            pose.pose.orientation.w = 1.0
            self.scene.add_box(FLOOR_NAME, pose, size=FLOOR_SIZE)
            self.get_logger().info("Added ground plane to planning scene.")
        self._floor_added = True

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(WORLD, CUBE_FRAME, rclpy.time.Time())
        except TransformException as ex:
            # waits until TF is available
            self.get_logger().debug(f"Waiting for TF {CUBE_FRAME}: {ex}")
            return

        pose = PoseStamped()
        pose.header.frame_id = WORLD
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation

        # upsert the cube as a collision box
        self.scene.add_box(OBJECT_NAME, pose, size=BOX_SIZE)

def main():
    rclpy.init()
    node = SceneSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
