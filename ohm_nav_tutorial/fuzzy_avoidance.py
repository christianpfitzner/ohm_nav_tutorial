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




    def f1(self, angle): 


        return 0.0

    def f2(self, angle):


        return 0.0
    



    def __scan_callback(self, msg):
        # create a obstacle avoidance algorithm
        
        cmd_vel = Twist()


        alpha = 0.0

        
        



        self.__publisher.publish(cmd_vel)





        




def main(args=None):
    rclpy.init(args=args)
    avoider = OhmObstacleAvoiderSimple()
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':

    main()


