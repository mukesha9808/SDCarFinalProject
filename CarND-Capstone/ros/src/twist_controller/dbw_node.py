#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
from scipy.spatial import KDTree

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        self.controller = Controller(vehicle_mass=vehicle_mass, 
                                     brake_deadband=brake_deadband, 
                                     decel_limit=decel_limit, 
                                     accel_limit=accel_limit,
                                     wheel_radius=wheel_radius, 
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio, 
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd',TwistStamped,self.twist_cb)
        rospy.Subscriber('/current_velocity',TwistStamped,self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.pose = None
        self.current_vel = None
        self.cur_ang_vel = None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.throttle = self.brake = self.steering = 0
        

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            if not None in (self.current_vel , self.linear_vel, self.cur_ang_vel , self.angular_vel, self.pose):
                closest_idx = self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y)
                waypoint=self.waypoints.waypoints[closest_idx]
                #rospy.loginfo('car position %s  and  %s', self.pose.pose.position.x,self.pose.pose.position.y)
                #rospy.loginfo('wp position %s  and  %s', waypoint.pose.pose.position.x,waypoint.pose.pose.position.y)
                cte = self.pose.pose.position.y - waypoint.pose.pose.position.y
                #rospy.loginfo('cte %s', cte)
                self.throttle, self.brake, self.steering = self.controller.control(self.dbw_enabled, self.current_vel, self.linear_vel , self.cur_ang_vel , self.angular_vel,cte) 
                
                

            if self.dbw_enabled :
                self.publish(self.throttle, self.brake, self.steering)
                rospy.loginfo('DBW status   %s',self.dbw_enabled)
            rate.sleep()

    def dbw_enabled_cb(self, msg) :
        self.dbw_enabled = msg
    
    def twist_cb(self, msg) :
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z
    
    def velocity_cb(self, msg) :
        self.current_vel = msg.twist.linear.x
        self.cur_ang_vel =msg.twist.angular.z
    
    
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)
   
    def pose_cb(self, msg):
        self.pose=msg
    
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d= [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree =KDTree(self.waypoints_2d)
    
    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx=self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

if __name__ == '__main__':
    DBWNode()
