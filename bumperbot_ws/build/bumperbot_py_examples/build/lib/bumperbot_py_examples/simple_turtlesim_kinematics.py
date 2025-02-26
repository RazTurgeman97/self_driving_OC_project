import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__('simple_turtlesim_kinematics')
        self.get_logger().info('Simple Turtlesim Kinematics node has been started')
        
        self.turtle1_pose_sub_ = self.create_subscription(Pose, "turtle1/pose", self.turtle1PoseCallback, 10)
        self.turtle2_pose_sub_ = self.create_subscription(Pose, "turtle2/pose", self.turtle2PoseCallback, 10)
        
        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

        self.create_timer(0.1, self.timer_callback)
        
    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_ = msg
        
    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_ = msg
        
        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y
        distance = (Tx**2 + Ty**2)**0.5
        self.get_logger().info("""\n
                               Transltation Vector turtle1 -> turtle2 \n
                               Tx: {Tx} \n
                               Ty: {Ty} \n
                               """)
        self.get_logger().info(f"Distance between turtles: {distance}")
        
def main():
    rclptr = rclpy.init()
    node = SimpleTurtlesimKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()