import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class MarkerListener(Node):
    def __init__(self):
        super().__init__('marker_listener')
        self.subscription = self.create_subscription(
            MarkerMessage,
            'aruco_markers',
            self.marker_callback,
            10
        )
        self.publisher = self.create_publisher(
            MarkerMessage,
            'robot_location',
            10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def marker_callback(self, msg):
        # Check if the marker_ids and poses lists have the same length
        if len(msg.marker_ids) != len(msg.poses):
            self.get_logger().warn("Number of marker_ids and poses don't match!")
            return

        # Find the index of the marker with id 0
        ref_marker_index = msg.marker_ids.index(0)

        # Get the pose of the reference marker
        ref_pose = msg.poses[ref_marker_index]

        # Create a new MarkerMessage to store the transformed poses
        robot_location_msg = MarkerMessage()
        robot_location_msg.header = msg.header
        robot_location_msg.marker_ids = []
        robot_location_msg.poses = []

        # Transform the poses of the other markers relative to the reference marker
        for i in range(len(msg.marker_ids)):
            if i != ref_marker_index:
                # Compute the relative pose
                relative_pose = Pose()
                relative_pose.position.x = msg.poses[i].position.x - ref_pose.position.x
                relative_pose.position.y = msg.poses[i].position.y - ref_pose.position.y
                relative_pose.position.z = msg.poses[i].position.z - ref_pose.position.z

                # Compute the relative orientation quaternion
                relative_orientation = self.compute_relative_orientation(ref_pose.orientation, msg.poses[i].orientation)
                relative_pose.orientation = relative_orientation

                # Add the marker id and transformed pose to the new message
                robot_location_msg.marker_ids.append(msg.marker_ids[i])
                robot_location_msg.poses.append(relative_pose)

        # Publish the transformed poses
        self.publisher.publish(robot_location_msg)

    def compute_relative_orientation(self, ref_orientation, orientation):
        # Convert the reference and target orientations to Euler angles
        ref_euler = self.quaternion_to_euler(ref_orientation)
        target_euler = self.quaternion_to_euler(orientation)

        # Compute the relative Euler angles
        relative_euler = [target - ref for target, ref in zip(target_euler, ref_euler)]

        # Convert the relative Euler angles back to a quaternion
        relative_orientation = self.euler_to_quaternion(relative_euler)

        return relative_orientation

    @staticmethod
    def quaternion_to_euler(quaternion):
        # Convert a quaternion to Euler angles
        import math

        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def euler_to_quaternion(euler):
        # Convert Euler angles to a quaternion
        import math
        import tf2_ros
        from geometry_msgs.msg import Quaternion

        roll, pitch, yaw = euler

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion = Quaternion()
        quaternion.w = cy * cp * cr + sy * sp * sr
        quaternion.x = cy * cp * sr - sy * sp * cr
        quaternion.y = sy * cp * sr + cy * sp * cr
        quaternion.z = sy * cp * cr - cy * sp * sr

        return quaternion


def main(args=None):
    rclpy.init(args=args)
    marker_listener = MarkerListener()
    rclpy.spin(marker_listener)
    marker_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
