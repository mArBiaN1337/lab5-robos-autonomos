import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose

class TurtleControlNode(Node):

    def __init__(self):
        self.goal_pose = None
        super().__init__('turtle_control')
        self.get_logger().info("Make Turtle go to GOAL pose!")
        self.init_variables()
        self.init_subscribers()
        self.init_publisher()

    def init_publisher(self):
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            "/turtle1/cmd_vel",
            10)
        
        timer_period = 0.5
        self.pub_callback_timer = self.create_timer(timer_period, self.pub_callback)
        

    def init_subscribers(self):
        self.pose_subscriber = self.create_subscription(
            Pose, 
            '/turtle1/pose', 
            self.pose_callback,
            10)
        
        self.goal_subscriber = self.create_subscription(
            Pose2D,
            '/marbian/goal',
            self.goal_callback,
            10)
        
    def init_goal_pose(self, msg: Pose):
        self.goal_pose = Pose()
        self.goal_pose.x = msg.x
        self.goal_pose.y = msg.y
        self.goal_pose.theta = msg.theta

    def init_variables(self):
        self.pose = Pose()
        self.flag = False
        self.k_omega = 1.5
        self.v_max = 1.5
    
    def pose_callback(self, msg : Pose):
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = msg.theta
        info = "[POSE_CALLBACK:[x:{:.3f},y:{:.3f},theta:{:.3f},".format(msg.x,msg.y,msg.theta) \
                + f"flag:{self.flag}]"
        self.get_logger().info(info)  
    
    def goal_callback(self, msg : Pose2D):
        self.init_goal_pose(msg)
    
    def pub_callback(self):
        if self.goal_pose is not None:
            self.flag = False
            x_err = self.goal_pose.x - self.pose.x
            y_err = self.goal_pose.y - self.pose.y

            p_ = math.sqrt(x_err**2 + y_err**2)
            alpha_ = math.atan2(y_err, x_err) - self.pose.theta

            dist_tol = 0.001
            ang_tol = 0.001

            vel_msg = Twist()
            if abs(alpha_) > ang_tol:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = self.k_omega * alpha_
            else:
                vel_msg.angular.z = 0.0
                if abs(p_) > dist_tol:
                    vel_msg.linear.x = self.v_max * p_
                else:
                    vel_msg.linear.x = 0.0
                    self.flag = True
                    self.get_logger().warn("Goal Reached!")

            self.cmd_vel_publisher.publish(vel_msg)
        else:
            self.get_logger().warn("Goal Not Set!")

def main(args=None):
    rclpy.init(args=args)

    turtle_ctrl = TurtleControlNode()
    rclpy.spin(turtle_ctrl)
    turtle_ctrl.destroy_node()  
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
