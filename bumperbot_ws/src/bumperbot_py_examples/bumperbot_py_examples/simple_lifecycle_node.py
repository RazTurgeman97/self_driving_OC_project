import rclpy
import rclpy.executors
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
import time



class SimpleLifecycleNode(Node):
    def __init__(self,node_name, **kwargs):
        
        super().__init__(node_name, **kwargs)
        self.get_logger().info("Simple Lifecycle Node has been created")
        
    def on_configured(self, state: State) -> TransitionCallbackReturn:
        
        self.sub_ = self.create_subscription(String, 'chatter', self.msgCallback, 10)
        self.get_logger().info("Lifecycle Node has been configured")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state):
        
        self.destroy_subscription(self.sub_)
        self.get_logger().info("Lifecycle Node has been shutdown")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleaningup(self, state: State) -> TransitionCallbackReturn:
        
        self.destroy_subscription(self.sub_)
        self.get_logger().info("Lifecycle Node has been cleaned up")
        return TransitionCallbackReturn.SUCCESS
    
    def on_activating(self, state: State) -> TransitionCallbackReturn:
        
        self.get_logger().info("Lifecycle Node has been activated")
        time.sleep(2)
        return super().on_activating(state)
    
    def on_deactivating(self, state: State) -> TransitionCallbackReturn:
        
        self.get_logger().info("Lifecycle Node has been deactivated")
        return super().on_deactivating(state)
    
    def msgCallback(self, msg):
        current_state = self._state_machine.current_state
        if current_state[1] == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info("Received message: " % msg.data)
        
        # self.get_logger().info(f"Received message: {msg.data}")
        
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = SimpleLifecycleNode('simple_lifecycle_node')
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()

    
if __name__ == '__main__':
    main()
        
    