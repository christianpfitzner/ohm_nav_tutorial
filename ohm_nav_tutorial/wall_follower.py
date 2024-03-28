# subscriber to the scan message

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import math


# create a publisher for the cmd_vel topic
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class OhmWallFollower(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_simple')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(LaserScan, 'scan', self.__scan_callback, 1)

        self.__right_corner = False

    def __scan_callback(self, msg):


        cmd_vel = Twist()

        idx_0  = len(msg.ranges)//2
        idx_90 = int(idx_0 - 90*math.pi / 180.0 // msg.angle_increment)
        idx_45 = int(idx_0 - 45*math.pi / 180.0 // msg.angle_increment)


        front_range       = msg.ranges[idx_0]
        right_range       = msg.ranges[idx_90]
        right_front_range = msg.ranges[idx_45]

        angle_45 = msg.angle_min - idx_45 * msg.angle_increment
        angle_90 = msg.angle_min - idx_90 * msg.angle_increment


        angle_45_in_degrees =  (msg.angle_min - angle_45) * 180.0 / math.pi 
        angle_90_in_degrees =  (msg.angle_min - angle_90) * 180.0 / math.pi 


        # calculate the distance to the wall
        d_a = math.sin(45*math.pi / 180.0) * right_front_range
        d_b = math.sin(90*math.pi / 180.0) * right_range


        if right_front_range > 1.0 and d_b < 1.0:
            self.__right_corner = True
            print("right corner")
        else:
            self.__right_corner = False
    

        # unit vector robot
        robot_unit_vector = Vector3()
        robot_unit_vector.x = 1.0       

        # vector wall
        wall_vector = Vector3()
        wall_vector.y = right_front_range * math.cos(angle_45) - right_range * math.cos(angle_90)
        wall_vector.x = right_front_range * math.sin(angle_45) - right_range * math.sin(angle_90)
        wall_vector.z = 0.0

        print(f"unit vector robot: {robot_unit_vector}")
        print(f"wall_vector: {wall_vector}")


        # calculate the angle between the robot and the wall
        angle = -math.acos((robot_unit_vector.x * wall_vector.x + robot_unit_vector.y * wall_vector.y) / (math.sqrt(robot_unit_vector.x**2 + robot_unit_vector.y**2) * math.sqrt(wall_vector.x**2 + wall_vector.y**2)))

        print(f"angle: {angle * 180.0 / math.pi}")

        # print(f"d_a: {d_a}, d_b: {d_b}")

        set_angle = 0.0


        

        if front_range < 1.0:
            cmd_vel.linear.x = 1.0 - front_range  # robot footprint
        else:
            cmd_vel.linear.x = 1.0

        # pd controller for the angle
        cmd_vel.angular.z = 5 * (set_angle - angle)

        if self.__right_corner:
            cmd_vel.angular.z = -4.0


        self.__publisher.publish(cmd_vel)





        




def main(args=None):
    rclpy.init(args=args)
    avoider = OhmWallFollower()
    rclpy.spin(avoider)
    avoider.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':

    main()


