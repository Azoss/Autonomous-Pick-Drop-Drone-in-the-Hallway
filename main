#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import tf
import cv2
import math
import numpy as np
import time
import RPi.GPIO as GPIO
import threading
from pynput import keyboard
from time import sleep
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, TwistStamped
from mavros_msgs.msg import PositionTarget, OverrideRCIn, RCIn, HomePosition, ActuatorControl
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode, CommandLong
from nav_msgs.msg import Odometry
import os.path
import subprocess

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        self.altitude = None
        #self.command_proxy = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        #rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose)         
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_data)
        rospy.Subscriber("/mavros/home_position/home", HomePosition, self.home_callback)
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped, self.altitude_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/distance_sensor/lidar_benawake_tf03", Range, self.range_callback)
        rospy.Subscriber("/mavros/distance_sensor/lidar_benawake_tfluna",Range, self.range_callback1)
        rospy.Subscriber("/mavros/distance_sensor/lidar_benawake_tfmini",Range,self.range_callback2)
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.cmd_range_pub = rospy.Publisher("/mavros/distance_sensor/lidarlite_pub",Range,queue_size=10)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.cmd_range_pub1 = rospy.Publisher("/mavros/distance_sensor/lidar_lidarlite_v3",Range, queue_size=10)
        self.servo_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
        self.pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        #self.set_yaw_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.set_yaw_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        
        self.rc = RCIn()
        self.rc3 = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()
        self.home = HomePosition()
        self.range = Range()
        self.range1 = Range()
        self.range2 = Range()
        self.range3 = Range()
        self.pose = PoseStamped()
        #self.nav = NavSatFix()
        current_pose = self.get_current_pose()
    def rc_data(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc3 = data.channels[7]
    def get_data(self):
        return self.rc3
    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data
    def home_callback(self, data):
        self.home = data
    def local_pose(self, data):
        self.pose1 = data.pose.position
        x = self.pose1.x 
        y = self.pose1.y 
        z = self.pose1.z 
        rospy.loginfo("FC Coordinates - X:{}, Y:{}, Z:{}".format(x, y, z))
        return x,y,z
    def get_coordinates(self):
        self.pose = PoseStamped()
        
        x = self.pose.pose.position.x - self.home.position.x
        y = self.pose.pose.position.y - self.home.position.y
        z = self.pose.pose.position.z - self.home.position.z
        rospy.loginfo("FC Coordinates - X:{}, Y:{}, Z:{}".format(x, y, z))
        return x, y, z
        
    def pose_callback(self, data):
        """
        Handle local position information
        """
        #self.timestamp = data.header.stamp
        self.pose = data.pose
    def get_current_pose(self):
        return self.pose
    
    def sensor (self):
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = "rangefinder"
        range_msg.radiation_type = Range.LIDAR
        range_msg.field_of_view = 0.1
        range_msg.min_range = 0.2
        range_msg.max_range = 4.0
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.sensor.publish(range_msg)
            rate.sleep()
        rospy.spin()
    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)
    def goto_range_pose(self, range):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = range

        self.cmd_pos_pub.publish(pose_stamped)
    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        #print(quat)
    def goto_zy(self, x, y, z):
        pose = Pose()
        current_pose = self.get_current_pose()  # Mendapatkan pose saat ini
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation = current_pose.orientation
        self.goto(pose)
        return True

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)
    def set_velmove(self, vy, vx, vz):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz        
        self.cmd_vel_pub.publish(cmd_vel)
        
    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        #mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        #mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp
        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()
    def move_drone1(self, velx, vely, velz, duration):
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            msg = PositionTarget()
            msg.header.stamp = self.timestamp
            msg.header.frame_id = "frame"
            msg.coordinate_frame = 8  # FRAME_BODY_OFFSET_NED pilihan 8/1
            msg.type_mask = 4039  # ignore orientation and thrust pilihan 4039 atau 4095
            msg.velocity.x = velx
            msg.velocity.y = vely
            msg.velocity.z = velz
            self.pub.publish(msg)

    def move_drone(self, vx, vy, vz):
        # Create the NED velocity vector.
        #ned_vec = np.array([[self.vel_x], [self.vel_y], [self.vel_z]], dtype=np.float32)
        msg = PositionTarget()
        msg.header.stamp = self.timestamp
        msg.header.frame_id = "frame"
        msg.coordinate_frame = 8 # FRAME_BODY_OFFSET_NED pilihan 8/1
        msg.type_mask = 4039 # ignore orientation and thrust pilihan 4039 atau 4095
        #msg = np.array(velocity)
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        self.pub.publish(msg)
    def send_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        rate = rospy.Rate(10)  # Frekuensi pengiriman pesan (10 Hz)
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(duration) 
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = rospy.Time.now()
            cmd_vel.twist.linear.x = velocity_x
            cmd_vel.twist.linear.y = velocity_y
            cmd_vel.twist.linear.z = velocity_z
            self.pub1.publish(cmd_vel)
            rate.sleep()    
    def altitude_callback(self, data):
        #self.altitude = PoseStamped()
        self.altitude = data.pose.position.z
        #rospy.loginfo("Altitude: {}".format(self.alt, '.4f'))
    def get_altitude(self):
        return self.altitude
    def range_callback(self,data):
        self.range = data.range 
    def get_range(self):
        return self.range
    def range_callback1(self,data):
        self.range1 = data.range
    def range_callback2(self,data):
        self.range2 = data.range
    def get_range1(self):
        return self.range1
    def get_range2(self):
        return self.range2
    def set_servo_channel(self, channel, value):
        """
        Mengatur nilai servo pada channel tertentu
        channel: nomor channel servo (0-7)
        value: nilai PWM servo (1000-2000)
        """
        rc_msg = OverrideRCIn()
        rc_msg.channels[channel] = value
        self.rc_override.publish(rc_msg)
        #return True
    def set_servo_value(self, servo_id, servo_value):
        command_msg = CommandLong()
        command_msg.command = 183  # MAV_CMD_DO_SET_SERVO
        command_msg.param1 = servo_id
        command_msg.param2 = servo_value
        command_msg.target_system = 1  # Set the target system ID
        command_msg.target_component = 1  # Set the target component ID

        response = self.command_proxy(command_msg)

        if not response.success:
            rospy.logwarn('Failed to send command: {}'.format(response.result))
    def move_to_position(self, servo_id, position):
        self.set_servo_value(servo_id, position)
    def trigger_servo_channel(self):
        actuator_msg = ActuatorControl()
        actuator_msg.header.stamp = rospy.Time.now()
        actuator_msg.group_mix = 0
        actuator_msg.controls[6] = 1700  # Channel 7 (AUX_OUT2) ke nilai PWM 1700
        self.servo_pub.publish(actuator_msg)
    def set_servo(self, channel, value):
        response = self.set_yaw_service(True, 183, 0, channel, value, 0, 0, 0, 0, 0)
        return True
      

range_kanan = 0
############################################
def keyboard_ditekan():
    # Mendengarkan input keyboard
    listener = keyboard.Listener(on_press=on_key_press)
    listener.start()
######################################
def on_key_press(keyboard):
    if keyboard == keyboard.KeyCode.from_char('t'):
        #lorong_maju1()
        print("###################oke masuk lorong#############")
##########################################
def motor_off():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(19, GPIO.OUT)
    GPIO.output(19, GPIO.HIGH)  # Matikan motor dengan mengubah pin menjadi LOW
    #GPIO.cleanup()
    return True  # Kembalikan nilai True untuk menunjukkan motor telah dimatikan
##################################################
def motor_on():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(19, GPIO.OUT)
    GPIO.output(19, GPIO.LOW)  # Matikan motor dengan mengubah pin menjadi LOW
    #GPIO.cleanup()
    return True  # Kembalikan nilai True untuk menunjukkan motor telah dimatikan
##################################################
def on_key_press(key):
        try:
            if key == keyboard.KeyCode.from_char('t'):
                lorong_maju1()
        except AttributeError:
            pass
def bebasnamanya():
    print("oke")
def on_key_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False
            
def panggil():
    listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
    listener.start()

    # Biarkan listener berjalan dalam background
    listener.join()
def lorong_maju():  ###############titik tengah 1.25 m
    global range_kanan
    mav = MavController()
    rospy.sleep(5)
    mav.takeoff(0.5)
    rospy.sleep(5)
    range_kanan = mav.get_range1()
    #range_depan = mav.get_range2()
    range_kanan_plus = range_kanan + 0.1 #################1.38
    range_kanan_min = range_kanan - 0.1 ############## 1.12
    range_kanan_plus1 = range_kanan + 0.2 ############1.5
    range_kanan_min1 = range_kanan - 0.2 ################## 1.0
    while not rospy.is_shutdown():
        range_kanan1 = mav.get_range1()
        range_depan1 = mav.get_range2()
        jarak = mav.get_range()
        print ("range kanan"+str(range_kanan1))
        print ("range depan"+str(range_depan1))
        print ("jarak"+str(jarak))
        if range_kanan1 < range_kanan_min1:
            if jarak >=0.4 and jarak <= 0.50:
                vx = 0.0
                vy = 0.2
                vz = 0.0
                mav.set_velmove(vx, vy, vz)
                print("ke kanan banyak")
                rospy.sleep(0.5)
            if jarak < 0.4:
                vx = 0.0
                vy = 0.2
                vz = 0.08
                mav.set_velmove(vx, vy, vz)
                print("kanan banyak naik")
                rospy.sleep(0.5)
            if jarak > 0.50:
                vx = 0.0
                vy = 0.2
                vz = -0.08
                mav.set_velmove(vx, vy, vz)
                print("ke kanan banyak turun")
                rospy.sleep(0.5)
        if range_kanan1 >= range_kanan_min1 and range_kanan1 <= range_kanan_min:
            if jarak >=0.4 and jarak <= 0.50:
                vx = 0.0
                vy = 0.08
                vz = 0
                mav.set_velmove(vx, vy, vz)
                print("ke kanan dikit")
                rospy.sleep(0.5)
            if jarak < 0.4:
                vx = 0.0
                vy = 0.08
                vz = 0.08
                mav.set_velmove(vx, vy, vz)
                print("ke kanan dikit naik")
                rospy.sleep(0.5)
            if jarak > 0.50:
                vx = 0.0
                vy = 0.08
                vz = -0.08
                mav.set_velmove(vx, vy, vz)
                print("ke kanan dikit turun")
                rospy.sleep(0.5)
        if range_kanan1 >= range_kanan_min and range_kanan1 <= range_kanan_plus:
            if jarak >=0.4 and jarak <= 0.50:
                vx = 0.15
                vy = 0
                vz = 0
                mav.set_velmove(vx, vy, vz)
                print("maju doang")
                rospy.sleep(0.7)
            if jarak < 0.4:
                vx = 0.15
                vy = 0
                vz = 0.08
                mav.set_velmove(vx, vy, vz)
                print("maju sambil naik")
                rospy.sleep(0.7)
            if jarak > 0.50:
                vx = 0.15
                vy = 0
                vz = -0.08
                mav.set_velmove(vx, vy, vz)
                print("maju sambil turun")
                rospy.sleep(0.7)
        if range_kanan1 >= range_kanan_plus and range_kanan1 <= range_kanan_plus1:
            if jarak >=0.4 and jarak <= 0.50:
                vx = 0.0
                vy = -0.08
                vz = 0
                mav.set_velmove(vx, vy, vz)
                print("ke kiri dikit")
                rospy.sleep(0.5)
            if jarak < 0.4:
                vx = 0.0
                vy = -0.08
                vz = 0.08
                mav.set_velmove(vx, vy, vz)
                print("ke kiri naik dikit")
                rospy.sleep(0.5)
            if jarak > 0.50:
                vx = 0.0
                vy = -0.08
                vz = -0.08
                mav.set_velmove(vx, vy, vz)
                print("ke kiri turun dikit")
                rospy.sleep(0.5)
        if range_kanan1 > range_kanan_plus1:
            if jarak >=0.4 and jarak <= 0.50:
                vx = 0.0
                vy = -0.2
                vz = 0.0
                mav.set_velmove(vx, vy, vz)
                print("ke kiri banyak")
                rospy.sleep(0.5)
            if jarak < 0.4:
                vx = 0.0
                vy = -0.2
                vz = 0.08
                mav.set_velmove(vx, vy, vz)
                print("ke kiri naik banyak")
                rospy.sleep(0.5)
            if jarak > 0.50:
                vx = 0.0
                vy = -0.2
                vz = -0.08
                mav.set_velmove(vx, vy, vz)
                print("ke kiri turun banyak")
                rospy.sleep(0.5)
        #if range_depan1 < 1.3:
        #    print("######masukkkkkk lorong kanan########")
        #    lorong_kanan()
        if range_depan1 <= 5.7:
            print("#################MASUK PICKMEE################")
            pick()
############################################     
def pick():
    mav = MavController()
    rospy.sleep(0.7)
    #mav.takeoff(0.6)
    #rospy.sleep(5)
    jarak = mav.get_range()
    range_kanan = 1.15 #1.12
    range_depan = 4.92 #5.0
    range_kanan_plus = range_kanan + 0.09 #0.065
    range_kanan_min = range_kanan - 0.09
    range_kanan_plus1 = range_kanan + 0.17 #0.065
    range_kanan_min1 = range_kanan - 0.17
    range_depan_plus = range_depan + 0.09
    range_depan_min = range_depan - 0.09
    #print("rangefinder_target_kanan=" + str(range_kanan))
    #print('rangefinder_target_depan='+str(range_depan))
    #panggil()
    #key = cv2.waitKey(1)
    while not rospy.is_shutdown():        
        range_kanan1 = mav.get_range1()
        range_depan1 = mav.get_range2()
        jarak = mav.get_range()
        #keyboard_ditekan()           
        if range_kanan1 < range_kanan_min1: #harus ke kirii dri dinding kiri dan maju dari dinding depan
            if range_depan1 < range_depan_min:
                if jarak >= 0.35  and jarak <= 0.45 :
                    #mav.move_drone(-0.06,0.1,0)
                    mav.set_velmove(-0.10  ,0.10 ,0)
                    print ("mundur kenceng")
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)             
                if jarak < 0.35 :
                    print("atas dulu baru kanan kenceng")
                    #mav.move_drone(0,0,0.10  )
                    mav.set_velmove(-0.10  ,0.10,0.08)
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)
                if jarak > 0.45:                   
                    mav.set_velmove(-0.10   ,0.10,-0.08)
                    print("atas dulu baru kanan kenceng")
                    rospy.sleep(0.7)
            if range_depan1 >= range_depan_min and range_depan1 <= range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.450 :
                    #mav.move_drone(0,0.10  ,0)
                    mav.set_velmove(0.0,0.10   ,0)
                    print ("kekanan kenceng")
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))    
                    rospy.sleep(0.7)          
                if jarak < 0.35:
                    print("atas dulu baru kanan kenceng")
                    mav.set_velmove(0,0.10   ,0.08)
                    #mav.move_drone(0,0,0.10  )
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)
                if jarak > 0.45:                   
                    mav.set_velmove(0,0.10   ,-0.08)
                    print("bawah dulu baru kanan kenceng")
                    rospy.sleep(0.7)
            if range_depan1 > range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    #mav.move_drone(0.10  ,0.10  ,0)
                    mav.set_velmove(0.10   ,0.10   ,0)
                    print (" maju kanan kenceng")
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)           
                if jarak < 0.35 :
                    print("atas dulu baru maju kanan kenceng")
                    mav.set_velmove(0.10   ,0.10   ,0.08)
                    #mav.move_drone(0,0,0.10  )
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)
                if jarak > 0.45:                   
                    mav.set_velmove(0.10   ,0.10   ,-0.08)
                    print("bawah dulu baru maju kanan kenceng")
                    rospy.sleep(0.7)
        if range_kanan1 >= range_kanan_min1 and range_kanan1 <= range_kanan_min:
            if range_depan1 > range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    #mav.move_drone(0.10  ,-0.10  ,0)
                    mav.set_velmove(0.05,0.05,0)
                    print ("maju kekanan kecil") 
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))  
                    rospy.sleep(0.7)               
                if jarak < 0.35 :
                    print("atas dulu baru maju kanan kecil")
                    #mav.move_drone(0,0,0.10  )
                    mav.set_velmove(0.05,0.05,0.08)
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)
                if jarak > 0.45:                   
                    mav.set_velmove(0.05,0.05,-0.08)
                    print("bawah dulu baru maju kanan kecil")
                    rospy.sleep(0.7)
            if range_depan1 >= range_depan_min and range_depan1 <= range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    #mav.move_drone(0,-0.10  ,0)
                    mav.set_velmove(0.0,0.05,0.0)
                    print ("kekanan") 
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))     
                    rospy.sleep(0.7)            
                if jarak < 0.35 :
                    print("atas dulu baru kanan")
                    mav.set_velmove(0.0,0.05,0.08)
                    #mav.move_drone(0,0,0.10  )
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)
                if jarak > 0.45:                   
                    mav.set_velmove(0.0,0.05,-0.08)
                    print("bawah dulu baru kanan")
                    rospy.sleep(0.7)
            if range_depan1 < range_depan_min:
                if jarak >= 0.35  and jarak <= 0.45 :
                    #mav.move_drone(-0.10  ,-0.10  ,0)
                    mav.set_velmove(-0.05,0,0)
                    print ("mundur") 
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)              
                if jarak < 0.35 :
                    print("atas dulu baru mundur")
                    mav.set_velmove(-0.05,0,0.08)
                    #mav.move_drone(0,0,0.10  )
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)
                if jarak > 0.45:                   
                    mav.set_velmove(-0.05,0,-0.08)
                    print("bawah dulu baru mundur kanan")
                    rospy.sleep(0.7)
        if range_kanan1 >= range_kanan_min and range_kanan1 <= range_kanan_plus:
            if range_depan1 > range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    #mav.move_drone(0.10  ,0,0)
                    mav.set_velmove(0.05,0.0,0)
                    print ("maju") 
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)                
                if jarak < 0.35 :
                    print("atas dulu baru maju")
                    #mav.move_drone(0,0,0.1)
                    mav.set_velmove(0.05,0,0.08)
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)
                if jarak > 0.45:                   
                    mav.set_velmove(0.05,0,-0.08)
                    print("bawah dulu baru maju")
                    rospy.sleep(0.7)
            if range_depan1 < range_depan_min:
                if jarak >= 0.35  and jarak <= 0.45 :
                    #mav.move_drone(-0.1,0,0)
                    mav.set_velmove(0,-0.05,0)
                    print ("kiri") 
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)                
                if jarak < 0.35 :
                    print("atas dulu baru kiri")
                    #mav.move_drone(0,0,0.1)
                    mav.set_velmove(0,-0.05,0.08)
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)   
                if jarak > 0.45:                   
                    mav.set_velmove(0,-0.05,-0.08)
                    print("bawah dulu baru kiri")
                    rospy.sleep(0.7)
            if range_depan1 >= range_depan_min and range_depan1 <= range_depan_plus:
                if jarak > 0.45:                   
                    mav.set_velmove(0,0,-0.10  )
                    print("turun udh ditengah")
                    rospy.sleep(0.7)
                if jarak >= 0.34 and jarak <= 0.45 :
                    #mav.move_drone(0,0,-0.1)
                    mav.set_velmove(0,0,-0.10 )
                    print ("turun")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7) 
                    motor_on()            
                if jarak < 0.34:                  
                    print("pick/////////////////")
                    vx = 0.0
                    vy = 0.0
                    vz = 0.0
                    #mav.move_drone(vx, vy, vz)
                    mav.set_velmove(vy, vx, vz)
                    rospy.sleep(0.7)
                    siap_naik = True
                    if siap_naik == True:
                        mav.send_velocity(0, 0, 0.2, 1.5)
                        #mav.send_velocity(0, 0.1, 0, 3.3)
                        print("naik")
                        lorong_maju1()
        if range_kanan1 >= range_kanan_plus and range_kanan1 <= range_kanan_plus1:
            if range_depan1 > range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    mav.set_velmove(0.05,-0.05,0.0)
                    print ("kiri maju")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)  
                if jarak <0.35 :
                    mav.set_velmove(0.05,-0.05,0.08)
                    print ("kiri maju naik")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)
                if jarak > 0.45:
                    mav.set_velmove(0.05,-0.05,-0.08)
                    print ("kiri maju")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)
            if range_depan1 < range_depan_min:
                if jarak >= 0.35  and jarak <= 0.45 :
                    mav.set_velmove(-0.06,0,0.0)
                    print ("kiri mundur")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))  
                    rospy.sleep(0.7) 
                if jarak <0.35 :
                    mav.set_velmove(-0.05,0,0.08)
                    print ("kiri mundur naik")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)
                if jarak > 0.45:
                    mav.set_velmove(-0.05,0,-0.08)
                    print ("kiri mundur turun")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))  
                    rospy.sleep(0.7) 
            if range_depan1 >= range_depan_min and range_depan1 <= range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    mav.set_velmove(0.0,-0.05,0.0)
                    print ("kiri aja")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)
                if jarak <0.35 :
                    mav.set_velmove(0.0,-0.05,0.08)
                    print ("kiri aja naik")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)
                if jarak > 0.45:
                    mav.set_velmove(0.0,-0.05,-0.08)
                    print ("kiri aja turun")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)         
        if range_kanan1 > range_kanan_plus1:
            if range_depan1 < range_depan_min:
                if jarak >= 0.35  and jarak <= 0.45 :
                    mav.set_velmove(-0.10   ,-0.1,0.0)
                    print ("mundur aja")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)
                if jarak <0.35 :
                    mav.set_velmove(-0.10   ,-0.1,0.08)
                    print ("mundur aja naik")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)
                if jarak > 0.45:
                    mav.set_velmove(-0.10   ,-0.1,-0.08)
                    print ("mundur aja turun")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))   
                    rospy.sleep(0.7)  
            if range_depan1 >= range_depan_min and range_depan1 <= range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    mav.set_velmove(0.0,-0.1 ,0.0)
                    print (" kiri aja kenceng")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)  
                if jarak <0.35 :
                    mav.set_velmove(0.0,-0.1 ,0.08)
                    print ("kiri aja naik")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)
                if jarak > 0.45:
                    mav.set_velmove(0.0,-0.1 ,-0.08)
                    print ("kiri aja turun")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak)) 
                    rospy.sleep(0.7)
            if range_depan1 > range_depan_plus:
                if jarak >= 0.35  and jarak <= 0.45 :
                    mav.set_velmove(0.10   ,-0.1  ,0.0)
                    print (" maju kiri aja kenceng")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)   
                if jarak <0.35 :
                    mav.set_velmove(0.10   ,-0.10   ,0.08)
                    print ("kiri maju aja naik")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7) 
                if jarak > 0.45:
                    mav.set_velmove(0.10   ,-0.10 ,-0.08)
                    print ("kiri maju aja turun")  
                    print("rangefinder=" + str(range_kanan1))
                    print("depan"+str(range_depan1))
                    print("alt"+str(jarak))
                    rospy.sleep(0.7)
                    
####################################
def lorong_maju1():  ###############titik tengah 1.25 m
    global range_kanan
    mav = MavController()
    rospy.sleep(1)
    #range_kanan = mav.get_range1()
    #range_depan = mav.get_range2()
    range_kanan_plus = range_kanan + 0.13 #################1.38
    range_kanan_min = range_kanan - 0.13 ############## 1.12
    range_kanan_plus1 = range_kanan + 0.25 ############1.5
    range_kanan_min1 = range_kanan - 0.25 ################## 1.0
    while not rospy.is_shutdown():
        range_kanan1 = mav.get_range1()
        range_depan1 = mav.get_range2()
        jarak = mav.get_range()
        print ("range kanan"+str(range_kanan1))
        print ("range depan"+str(range_depan1))
        print ("jarak"+str(jarak))
        if range_depan1 < 1.7:
            lorong_kanan()
            print("masuk lorong kanannnnnn")
        if range_depan1 == 0:
            print("land and relay off")
            mav.land()
            motor_off()
        if range_kanan1 < range_kanan_min1:
            if jarak >=0.55 and jarak <= 0.75:
                vx = 0.0
                vy = 0.2
                vz = 0.0
                mav.set_velmove(vx, vy, vz)
                print("ke kanan banyak")
                rospy.sleep(0.5)
            if jarak < 0.55:
                vx = 0.0
                vy = 0.2
                vz = 0.1
                mav.set_velmove(vx, vy, vz)
                print("kanan banyak naik")
                rospy.sleep(0.5)
            if jarak > 0.75:
                vx = 0.0
                vy = 0.2
                vz = -0.1
                mav.set_velmove(vx, vy, vz)
                print("ke kanan banyak turun")
                rospy.sleep(0.5)
        if range_kanan1 >= range_kanan_min1 and range_kanan1 <= range_kanan_min:
            if jarak >=0.55 and jarak <= 0.75:
                vx = 0.0
                vy = 0.1
                vz = 0
                mav.set_velmove(vx, vy, vz)
                print("ke kanan dikit")
                rospy.sleep(0.5)
            if jarak < 0.55:
                vx = 0.0
                vy = 0.1
                vz = 0.1
                mav.set_velmove(vx, vy, vz)
                print("ke kanan dikit naik")
                rospy.sleep(0.5)
            if jarak > 0.75:
                vx = 0.0
                vy = 0.1
                vz = -0.1
                mav.set_velmove(vx, vy, vz)
                print("ke kanan dikit turun")
                rospy.sleep(0.5)
        if range_kanan1 >= range_kanan_min and range_kanan1 <= range_kanan_plus:
            if jarak >=0.55 and jarak <= 0.75:
                vx = 0.3
                vy = 0
                vz = 0
                mav.set_velmove(vx, vy, vz)
                print("maju doang")
                rospy.sleep(0.5)
            if jarak < 0.55:
                vx = 0.3
                vy = 0
                vz = 0.1
                mav.set_velmove(vx, vy, vz)
                print("maju sambil naik")
                rospy.sleep(0.5)
            if jarak > 0.75:
                vx = 0.3
                vy = 0
                vz = -0.1
                mav.set_velmove(vx, vy, vz)
                print("maju sambil turun")
                rospy.sleep(0.5)
        if range_kanan1 >= range_kanan_plus and range_kanan1 <= range_kanan_plus1:
            if jarak >=0.55 and jarak <= 0.75:
                vx = 0.0
                vy = -0.1
                vz = 0
                mav.set_velmove(vx, vy, vz)
                print("ke kiri dikit")
                rospy.sleep(0.5)
            if jarak < 0.55:
                vx = 0.0
                vy = -0.1
                vz = 0.1
                mav.set_velmove(vx, vy, vz)
                print("ke kiri naik dikit")
                rospy.sleep(0.5)
            if jarak > 0.75:
                vx = 0.0
                vy = -0.1
                vz = -0.1
                mav.set_velmove(vx, vy, vz)
                print("ke kiri turun dikit")
                rospy.sleep(0.5)
        if range_kanan1 > range_kanan_plus1:
            if jarak >=0.55 and jarak <= 0.75:
                vx = 0.0
                vy = -0.2
                vz = 0.0
                mav.set_velmove(vx, vy, vz)
                print("ke kiri banyak")
                rospy.sleep(0.5)
            if jarak < 0.55:
                vx = 0.0
                vy = -0.2
                vz = 0.1
                mav.set_velmove(vx, vy, vz)
                print("ke kiri naik banyak")
                rospy.sleep(0.5)
            if jarak > 0.75:
                vx = 0.0
                vy = -0.2
                vz = -0.1
                mav.set_velmove(vx, vy, vz)
                print("ke kiri turun banyak")
                rospy.sleep(0.5)
###################################
def lorong_kanan():
    mav = MavController()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        range_kanan = mav.get_range1()
        range_depan = mav.get_range2()
        jarak = mav.get_range()
        if range_kanan >= 1.0 and range_kanan <= 3.7:
            if range_depan >= 1.2 and range_depan <= 1.4: #ditengah
                if jarak >=0.75 and jarak <= 0.85:
                    mav.move_drone(0, -0.35,0)
                    print("pas ditengah + gerak kanan")
                    rospy.sleep(0.5)
                if jarak < 0.75:
                    mav.move_drone(0,-0.35,0.1)
                    print("terlalu turun, naik sambil kanan")
                    rospy.sleep(0.5)
                if jarak > 0.85:
                    mav.move_drone(0,-0.35,-0.1)
                    print("terlalu naik, turun sambil kanan")
                    rospy.sleep(0.5)
            if range_depan < 1.2: #harus gerak ke kanan
                if jarak >=0.75 and jarak <= 0.85:
                    mav.move_drone(-0.15,-0.35, 0)
                    print("terlalu maju, ke kanan sambil mundur")
                    rospy.sleep(0.5)
                if jarak < 0.75:
                    mav.move_drone(-0.15,-0.35,0.1)
                    print("terlalu maju dan kurang naik, ke kanan sambil mundur dan naik")
                    rospy.sleep(0.5)
                if jarak > 0.85:
                    mav.move_drone(-0.15,-0.35,-0.1)
                    print("terlalu maju dan ketinggian, ke kanan sambil mundur dan turun")
                    rospy.sleep(0.5)
            if range_depan > 1.4: #harus ke kiri
                if jarak >=0.75 and jarak <= 0.85:
                    mav.move_drone(0.15, -0.35, 0)
                    print("maju sambil kanan")
                    rospy.sleep(0.5)
                if jarak < 0.75:
                    mav.move_drone(0.1, -0.35, 0.1)
                    print("maju sambil kanan dan naik")
                    rospy.sleep(0.5)
                if jarak > 0.85:
                    mav.move_drone(0.1, -0.35, -0.1)
                    print("maju sambil kanan dan turun")
                    rospy.sleep(0.5)
            if range_depan == 0:
                motor_off()
                mav.land()
                print("land")
        if range_kanan < 1.0:            
            mav.move_drone(0, -0.35, 0)
            print("ke kanan woi")
            rospy.sleep(0.5)
        if range_kanan > 3.7:
            if range_depan >= 1.2 and range_depan <= 1.4:
                if jarak >=0.75 and jarak <= 0.85: ##############################
                    mav.move_drone(0, 0,0)
                    webcam()
                    print("masuk webcam")
                if jarak < 0.75:
                    mav.move_drone(0,-0.15,0.1)
                    print("terlalu turun, naik sambil kanan")
                    webcam()
                    print("masuk webcam")
                if jarak > 0.85:
                    mav.move_drone(0,-0.15,-0.1)
                    print("terlalu naik, turun sambil kanan")
                    webcam()
                    print("masuk webcam")
            if range_depan < 1.2: #harus gerak ke kanan
                if jarak >=0.75 and jarak <= 0.85:
                    mav.move_drone(-0.15,-0.15, 0)
                    print("terlalu maju, ke kanan sambil mundur")
                    rospy.sleep(0.5)
                if jarak < 0.75:
                    mav.move_drone(-0.1,-0.15,0.1)
                    print("terlalu maju dan kurang naik, ke kanan sambil mundur dan naik")
                    rospy.sleep(0.5)
                if jarak > 0.85:
                    mav.move_drone(-0.1,-0.15,-0.1)
                    print("terlalu maju dan ketinggian, ke kanan sambil mundur dan turun")
                    rospy.sleep(0.5)
            if range_depan > 1.4: #harus ke kiri
                if jarak >=0.75 and jarak <= 0.85:
                    mav.move_drone(0.15, -0.15, 0)
                    print("maju sambil kanan")
                    rospy.sleep(0.5)
                if jarak < 0.75:
                    mav.move_drone(0.1, -0.15, 0.1)
                    print("maju sambil kanan dan naik")
                    rospy.sleep(0.5)
                if jarak > 0.85:
                    mav.move_drone(0.1, -0.15, -0.1)
                    print("maju sambil kanan dan turun")
                    rospy.sleep(0.5)
####################################
def webcam():
    mav = MavController()
    rospy.sleep(0.5)
    image_pub = rospy.Publisher('/webcamdrop', Image, queue_size=10)
    image_pub2 = rospy.Publisher('/maskdrop', Image, queue_size=10)
    bridge = CvBridge()
    ###################################
    hfov = 60
    hres = 720
    vfov = 60
    vres = 720
    font = cv2.FONT_HERSHEY_COMPLEX
    ###################################
    def pixels_per_meter(fov, res, alt):
        return ((alt * math.tan(math.radians(fov / 2))) / (res / 2))
    #######################
    def destroy_rqt_image_view():
        try:
            subprocess.Popen(['pkill', 'rqt_image_view'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print("rqt_image_view telah ditutup.")
        except Exception as e:
            print("Error:", e)  
    ###########################
    def disable_video0():
        try:
            subprocess.Popen(['v4l2-ctl', '-d', '/dev/video0', '-c', 'video_power_mode=0'], shell=True,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print("Video2 disabled.")
        except Exception as e:
            print("Error:", e)
    # Camera setup##############
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
    subprocess.Popen("rqt_image_view /webcamdrop", shell=True)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue
        altitude = mav.get_altitude()
        jarak = mav.get_range()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        range_kanan = mav.get_range1()
        range_depan = mav.get_range2()
        # Threshold the HSV image to get only green colors        
        #lower_red = (45, 150, 15)
        #upper_red = (70, 255, 255)
        ########dikons#######
        #lower_red = (0, 67, 0)
        #upper_red = (255, 255, 255)
        #####pagi ke siang###
        lower_red = (137, 84, 0)
        upper_red = (255, 255, 255)
        #####jam 9#########
        #lower_red = (0, 98, 45)
        #upper_red = (255, 255, 255)
        mask = cv2.inRange(hsv_frame, lower_red, upper_red)
        cv2.line(frame, (350, 320), (370, 320), (0, 255, 255), 2)  # + horizontal line
        cv2.line(frame, (360, 310), (360, 330), (0, 255, 255), 2)  # + vertical line
        cv2.rectangle(frame, (330, 340), (390, 400), (178, 54, 89), 2)
        cv2.circle(frame, (int(360), int(370)), int(50), (0, 255, 255), 2)
        x = 360
        y = 370
        #line segment
        # Line segment garis vertikal 1
        cv2.line(frame, (303, 0), (303, 720), (0, 255, 255), 2)

        # Line segment garis vertikal 2
        cv2.line(frame, (476, 0), (476, 720), (0, 255, 255), 2)

        # Line segment garis horizontal 1
        cv2.line(frame, (0, 213), (640, 213), (0, 255, 255), 2)

        # Line segment garis horizontal 2
        cv2.line(frame, (0, 426), (640, 426), (0, 255, 255), 2)
        
        if range_kanan >= 1.0:
            #if range_depan >= 1.1 and range_depan <= 1.5: #ditengah
            if jarak >=0.75 and jarak <= 0.85:
                mav.move_drone(0, -0.2,0)
                #rospy.sleep(0.3)
            if jarak < 0.75:
                mav.move_drone(0,-0.15,0.15)
                #rospy.sleep(0.3)
            if jarak > 0.85:
                mav.move_drone(0,-0.15,-0.15)
                #rospy.sleep(0.3)  
        if range_kanan < 1.0:            
            mav.move_drone(0, -0.2, 0)
            print("ke kanan woi")
            #rospy.sleep(0.3)

        contour, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contour:
            area = cv2.contourArea(cnt)
            epsilon = 0.01 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            if area > 11000:
                # cv.drawContours(frame, [approx], 0, (0, 255, 0), 3)
                c = approx
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                # Calculate the center of the contour
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    # Handle the case when m00 is zero (avoid division by zero)
                    center = (0, 0)

                #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 130:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.line(frame, (int(x), int(y)), (360, 370), (0, 255, 255), 2)
                    center_rect = (x, y)
                    target_cent=(360,370)
                    #batas
                    kx = x
                    ky = y
                    # target-center
                    x -= 360
                    y = 370 - y
                    px_meter_x = pixels_per_meter(hfov, hres, jarak)
                    px_meter_y = pixels_per_meter(vfov, vres, jarak)

                    x = -x * px_meter_x * 0.3
                    y = y * px_meter_y * 0.3
                    vx = round(y, 3)
                    vy = round(x, 3)
                    vz= 0
                    if kx > 330 and kx < 390:
                        if ky > 340 and ky < 400:
                            vx = 0
                            vy = 0
                            vz = 0
                            disable_video0()
                            cap.release()
                            motor_off()
                            destroy_rqt_image_view()                  
                            #break
                    print("AREA == " + str(area) + (" PIXEL"))
                    print("vx = " + str(vx))
                    print("vy = " + str(vy))
                    print("rangefinder bawah=" + str(jarak))
                    print("radius=" + str(radius))
                    mav.move_drone(vx, vy, vz)
                    rospy.sleep(0.1)
        fps = cap.get(cv2.CAP_PROP_FPS)
        afs = str("Fps {0}".format(fps))
        cv2.putText(frame, afs, (10, 60), font, 1, (255,255,255), 1)
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            image_pub2.publish(bridge.cv2_to_imgmsg(mask, "mono8"))
        except CvBridgeError as e:
            print(e)

###############################
def utama():
    mav = MavController()
    rospy.sleep(1)
    #low = mav.get_data()
    while not rospy.is_shutdown():
        low = mav.get_data()
        print("channel 8 = "+str(low))
        if low == 982:
            print("masuk kamera")
            lorong_maju()
        else:
            print("low dulu bro")
            rospy.sleep(1)
            #break #3.33 , 1.52
########################  
def simple():
    def on_key_press(key):
        try:
            if key == keyboard.KeyCode.from_char('t'):
                print("T key pressed")
        except AttributeError:
            pass

    def on_key_release(key):
        if key == keyboard.Key.esc:
            # Stop listener
            return False
            
    def panggil():
        listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
        listener.start()

        # Biarkan listener berjalan dalam background
        listener.join()
    while True: 
        print("okeee")     
        panggil()
##########################          
if __name__=="__main__":
    #camera()
    utama()
    #pick()
    #camera()
    #motor_on()
    #motor_off()
    #GPIO.cleanup()
    #webcam()
    #simple()
    #lorong_maju()
    #simple()
