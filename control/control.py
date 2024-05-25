import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float32

class Control(Node):
    def __init__(self):
        super().__init__('control')
        self.left_motor_speed_publisher = self.create_publisher(Float32, '/robot/chassis/left_motor/target_speed', 10)
        self.right_motor_speed_publisher = self.create_publisher(Float32, '/robot/chassis/right_motor/target_speed', 10)

        self.target_movement_subscriber = self.create_subscription(Float32MultiArray, 'target_movement', self.speed_compute, 10)

    def speed_compute(self, msg):
        v_foward = msg.data[0]
        angular = msg.data[1]

        speed_left = (angular * (0.49/2)) + v_foward
        speed_right = 2*v_foward - speed_left

        self.speed_publish(speed_left, speed_right)

    def speed_publish(self, speed_left, speed_right):
        speed_msg_left = Float32()
        speed_msg_right = Float32()

        speed_msg_left.data = speed_left
        speed_msg_right.data = speed_right

        self.left_motor_speed_publisher.publish(speed_msg_left)
        self.right_motor_speed_publisher.publish(speed_msg_right)


        
                
def main(args=None):
    rclpy.init(args=args)

    control = Control()
    rclpy.spin(control)


    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()