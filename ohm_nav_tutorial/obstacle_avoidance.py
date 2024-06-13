# subscriber to the scan message

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# create a publisher for the cmd_vel topic
from geometry_msgs.msg import Twist

class OhmObstacleAvoiderSimple(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_simple')

        self.__publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 1)

        self.create_subscription(LaserScan, '/robot1/laser', self.__scan_callback, 1)



    def __scan_callback(self, msg):
        # create a obstacle avoidance algorithm
        
        #get the middle range of the scan
        middle_range = msg.ranges[len(msg.ranges)//2]

        # get the range of 45 degrees to the left
        right_range = msg.ranges[len(msg.ranges)//4]

        # get the range of 45 degrees to the right
        left_range = msg.ranges[3*len(msg.ranges)//4]

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3
        cmd_vel.angular.z = 0.0



        # # if the right range is smaller than 0.5 meters turn left
        # if right_range < 1.0:
        #     cmd_vel.angular.z = -1.3
        # elif left_range < 1.0:
        #     cmd_vel.angular.z = 1.3


        cmd_vel.angular.z = (left_range - right_range) * 0.5


        if middle_range < 1.5:
            cmd_vel.angular.z = 1.5 - middle_range - 0.6 # robot footprint

        print(f"middle range: {middle_range}, left range: {left_range}, right range: {right_range}")

        self.__publisher.publish(cmd_vel)





        




def main(args=None):
    rclpy.init(args=args)
    avoider = OhmObstacleAvoiderSimple()
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':

    main()


