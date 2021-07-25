import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit,
                wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        #rospy.loginfo('wheel_base  %s', wheel_base)
        #rospy.loginfo('steer_ratio  %s', steer_ratio)
        #rospy.loginfo('max_lat_accel  %s', max_lat_accel)
        #rospy.loginfo('max_steer_angle  %s', max_steer_angle)
        
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        kp=0.035
        ki=0.0002
        kd=0
        mn=0.
        mx=0.5
        
        self.pedal_controller = PID( kp, ki, kd, mn, mx)
        self.pedal_controller.reset()
        
        self.steer_controller = PID( 5, 0.00002,400, -max_steer_angle, max_steer_angle)
        self.steer_controller.reset()
        
        tau=0.5
        ts=0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
        self.steering = 0


    def control(self, dbw_enabled, current_vel, linear_vel, cur_ang_vel, angular_vel, cte):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.pedal_controller.reset()
            return 0., 0., 0.
        
        current_vel = self.vel_lpf.filt(current_vel)
        
        
        steererror=angular_vel-cur_ang_vel
        if (abs(steererror) < 0.001):
            steering=0
        else:
            steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        #rospy.loginfo('Yaw_lin    %s',linear_vel)
        #rospy.loginfo('yaw_ang    %s',angular_vel)
        #rospy.loginfo('Yaw_curang    %s',cur_ang_vel)
        #rospy.loginfo('Yaw_curvel    %s',current_vel)
        
        
        vel_error = linear_vel - current_vel
        
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
         
        throttle = self.pedal_controller.step(vel_error, sample_time)
        #rospy.loginfo('error    %s',vel_error)
        #rospy.loginfo('pedal    %s',throttle)
        
        #rospy.loginfo('steererror    %s',steererror)
        #rospy.loginfo('steering    %s',steering)
        brake=0
        
        if linear_vel==0 and current_vel <0.1 :
            throttle=0
            brake=700
            rospy.loginfo('veh Stopping')
            
        elif throttle <0.1 and vel_error < 0:
            throttle=0
            decel=max(vel_error, self.decel_limit)
            brake= abs(decel)* self.vehicle_mass *self.wheel_radius
            rospy.loginfo('veh slowing')
        
        return throttle, brake, steering
                
        
