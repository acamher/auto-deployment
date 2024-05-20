 #!/usr/bin/python3
#   !/usr/bin/env python
import time
import cv2
import threading
from std_msgs.msg import Float64
#from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import rospy
import math
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import RCOut
import RPi.GPIO as GPIO
import pigpio
import sys

rc_sim = False 

GPIO.setwarnings(False)
GPIO.cleanup()

class Raspi:
    def __init__(self):
         #TEAM MASI INITIALIZATION
        self.check_deploy1 = 0
        self.check_deploy2 = 0
        self.RAW_CHAN1_RAW = 0
        self.RAW_CHAN2_RAW = 0
        self.RAW_CHAN3_RAW = 0
        self.VIDEO_SWITCHER_SWC = 0
        self.hd_cam = 1000
        self.thermal_cam = 1300
        self.gimb_cam = 1800
        self.rcout_cam = 0


        #MAVROS VARIABLES
        self.altitude = 0.0
        self.speed = 0.0
        self.longitude = 0.0
        self.latitude = 0.0
        self.rc1 = 0
        self.rc2 = 0
        self.rc3 = 0
        self.rc4 = 0
        self.rc5 = 0
        self.rc6 = 0
        self.rc7 = 0
        self.rc8 = 0
        self.rc9 = 0
        self.rc10 = 0
        self.rc11 = 0
        self.rc12 = 0
        self.rc13 = 0
        self.rc14 = 0
        self.rc15 = 0
        self.rc16 = 0
        self.rcout1 = 0
        self.rcout2 = 0
        self.rcout3 = 0
        self.rcout4 = 0
        self.rcout5 = 0
        self.rcout6 = 0
        self.rcout7 = 0
        self.rcout8 = 0
        self.state = False
        self.hdg = 0
        self.battery_per = 100
        self.battery_vol = 16
        ##LAUNCHING CONTROL VARIABLES
        self.control = 7
        self.servo1open = True
        self.servo2open = True
        self.servo3open = True
        self.servo4open = True
        self.cam_error = False

        #Servos precambio
        self.pin_servo1 = 18
        self.pin_servo2 = 27
        self.pin_servo3 = 12
        self.pin_servo4 = 19
        self.pin_gimb_pitch = 5
        self.pin_gimb_yaw = 13
        self.pin_switch = 17

        self.pwm = pigpio.pi()
        self.pwm.set_mode(self.pin_servo1, pigpio.OUTPUT)
        self.pwm.set_mode(self.pin_servo2, pigpio.OUTPUT)
        self.pwm.set_mode(self.pin_servo3, pigpio.OUTPUT)
        self.pwm.set_mode(self.pin_servo4, pigpio.OUTPUT)
        self.pwm.set_mode(self.pin_gimb_pitch, pigpio.OUTPUT)
        self.pwm.set_mode(self.pin_gimb_yaw, pigpio.OUTPUT)
        self.pwm.set_mode(self.pin_switch, pigpio.OUTPUT)
        self.pwm.set_PWM_frequency( self.pin_servo1, 50 )
        self.pwm.set_PWM_frequency( self.pin_servo2, 50 )
        self.pwm.set_PWM_frequency( self.pin_servo3, 50 )
        self.pwm.set_PWM_frequency( self.pin_servo4, 50 )
        self.pwm.set_PWM_frequency( self.pin_gimb_pitch, 50 )
        self.pwm.set_PWM_frequency( self.pin_gimb_yaw, 50 )
        self.pwm.set_PWM_frequency( self.pin_switch, 50 )
        self.open1 = 1225
        self.close1 = 1000
        self.open2 = 1385
        self.close2 = 1675
        ######NEW SERVOS
        self.open3 = 1225    #cambiar este numero
        self.close3 = 1000    #cambiar este numero
        self.open4 = 1385     #cambiar este numero
        self.close4 = 1675    #cambiar este numero
        self.pitch_min = 1000
        self.pitch_max = 2300 #45 grados up
        self.pitch_horizontal = 1850
        self.yaw_min = 550
        self.yaw_max = 1900
        self.yaw_forward = 1275
        


    def position_callback(self, position_msg):
        try:
            self.altitude = position_msg.pose.pose.position.z
            self.speed = math.sqrt(position_msg.twist.twist.linear.x*position_msg.twist.twist.linear.x + position_msg.twist.twist.linear.y*position_msg.twist.twist.linear.y)
        except:
             self.altitude = 99
             self.speed = 13

    def navsat_callback(self, navsat_msg):
        try:
            self.longitude = navsat_msg.longitude
            self.latitude = navsat_msg.latitude
        except:
            self.longitude = 17
            self.latitude = 17

    def rc_callback(self, rc_msg):

            self.rc1 = int(rc_msg.channels[0])
            self.rc2 = int(rc_msg.channels[1])
            self.rc3 = int(rc_msg.channels[2])
            self.rc4 = int(rc_msg.channels[3])
            self.rc5 = int(rc_msg.channels[4])
            self.rc6 = int(rc_msg.channels[5])
            self.rc7 = int(rc_msg.channels[6])
            self.rc8 = int(rc_msg.channels[7])
            self.rc9 = int(rc_msg.channels[8])
            self.rc10 = int(rc_msg.channels[9])
            self.rc11 = int(rc_msg.channels[10])
            self.rc12 = int(rc_msg.channels[11])
            self.rc13 = int(rc_msg.channels[12])
            self.rc14 = int(rc_msg.channels[13])
            self.rc15 = int(rc_msg.channels[14])
            self.rc16 = int(rc_msg.channels[15])
            self.RAW_CHAN1_RAW = self.rc6  #LAUNCH1
            self.RAW_CHAN2_RAW = self.rc7  #LAUNCH2
            self.RAW_CHAN3_RAW = self.rc8  #ENVOLVENTE

            


    def rcout_callback(self, rc_msg):
            self.rcout1 = int(rc_msg.channels[0])
            self.rcout2 = int(rc_msg.channels[1])
            self.rcout3 = int(rc_msg.channels[2])
            self.rcout4 = int(rc_msg.channels[3])
            self.rcout5 = int(rc_msg.channels[4])
            self.rcout6 = int(rc_msg.channels[5])
            self.rcout7 = int(rc_msg.channels[6])
            self.rcout8 = int(rc_msg.channels[7])

            self.rcout_cam = self.rcout7%5
            self.rcout7-=self.rcout_cam
            self.pitch_control(self.rcout7)
            self.yaw_control(self.rcout8)

    def pitch_control(self,val):
        self.pwm.set_servo_pulsewidth( self.pin_gimb_pitch, val ) 

    def yaw_control(self,val): 
        self.pwm.set_servo_pulsewidth(self.pin_gimb_yaw, val ) 

    def move(self,servo, val):
        self.pwm.set_servo_pulsewidth( servo, val ) 
        time.sleep(0.8)
        #self.pwm.set_servo_pulsewidth(servo, 0)
   
    def compass_hdg_callback(self,compass_hdg_msg):
        try:
            #self.hdg = int(compass_hdg_msg.data)
            self.hdg = int(compass_hdg_msg)
        except:
            self.hdg= 0

        #print(self.hdg)

    def battery_callback(self,battery_msg):
        try:
            self.battery_per = int(battery_msg.percentage)
            self.battery_vol = int(battery_msg.voltage)
        except:
            self.battery_per = 100
            self.battery_vol = 24

    def launcher_old(self):

       # self.move(self.pin_servo1,self.open1)
       # self.move(self.pin_servo2,self.open2)
        self.pitch_control(self.pitch_horizontal)
        self.yaw_control(self.yaw_forward)
        while True:


            if self.RAW_CHAN3_RAW > 1800: # envolvente activada, solo se lanza el dron si esta en la envolvente de velocidad y altura
                if (self.altitude >= 40 and self.speed <= 35):
                    if self.RAW_CHAN1_RAW > 1200:
                        if self.servo1open == True:
                            self.move(self.pin_servo1,self.close1)
                            self.servo1open = False
                    else:
                        if self.servo1open == False:
                            self.move(self.pin_servo1,self.open1)
                            self.servo1open = True
                    if self.RAW_CHAN2_RAW > 1200:
                        if self.servo2open == True:
                            self.move(self.pin_servo2,self.close2)
                            self.servo2open = False
                    else:
                        if self.servo2open == False:
                            self.move(self.pin_servo2,self.open2)
                            self.servo2open = True
            elif (self.RAW_CHAN3_RAW>1400): #Envolvente desactivada
                if self.RAW_CHAN1_RAW > 1200:
                    if self.servo1open == True:
                        self.move(self.pin_servo1,self.close1)
                        self.servo1open = False
                else:
                    if self.servo1open == False:
                        self.move(self.pin_servo1,self.open1)
                        self.servo1open = True
                if self.RAW_CHAN2_RAW > 1200:
                    if self.servo2open == True:
                        self.move(self.pin_servo2,self.close2)
                        self.servo2open = False
                else:
                    if self.servo2open == False:
                        self.move(self.pin_servo2,self.open2)
                        self.servo2open = True
            else:  #JETTISON
                #print("JETT")
                if self.servo1open == False:
                    self.move(self.pin_servo1,self.open1)
                    self.servo1open = True

                if self.servo2open == False:
                    self.move(self.pin_servo2,self.open2)
                    self.servo2open = True
    
    ####2023/06/01 LAUNCH FUNCTION
    def launcher(self):

        while True:
            if self.RAW_CHAN3_RAW > 1800: # envolvente activada, solo se lanza el dron si esta en la envolvente de velocidad y altura
                if (self.altitude >= 40 and self.speed <= 35):
                   self.servo_behaviour()
            elif (self.RAW_CHAN3_RAW>1400): #Envolvente desactivada
                #CHESER 1 Y 2
                    self.servo_behaviour()
            else:  #JETTISON
                #print("JETT")
                if self.servo1open == False:
                    self.move(self.pin_servo1,self.open1)
                    self.servo1open = True

                if self.servo2open == False:
                    self.move(self.pin_servo2,self.open2)
                    self.servo2open = True

                if self.servo3open == False:
                    self.move(self.pin_servo3,self.open3)
                    self.servo3open = True

                if self.servo4open == False:
                    self.move(self.pin_servo4,self.open4)
                    self.servo4open = True

            if (self.rc12 < 1100):
                self.VIDEO_SWITCHER_SWC = 0
            elif (self.rc12 < 1700):
                self.VIDEO_SWITCHER_SWC = 2
            else:
                self.VIDEO_SWITCHER_SWC = 1
            if(self.rcout_cam>0):
                if (self.rcout_cam < 2):
                    self.VIDEO_SWITCHER_SWC = 0
                elif (self.rcout_cam < 3):
                    self.VIDEO_SWITCHER_SWC = 2
                else:
                    self.VIDEO_SWITCHER_SWC = 1

            if (self.VIDEO_SWITCHER_SWC != self.control):
                self.switcher(self.VIDEO_SWITCHER_SWC)
                self.control = self.VIDEO_SWITCHER_SWC

    def servo_behaviour(self):
                            #CHASER 1 Y 2

                    if self.RAW_CHAN1_RAW < 1300:  ###OPEN SERVO2 AND CLOSE SERVO1
                        if self.servo1open == True:  
                            self.move(self.pin_servo1,self.close1)
                            self.servo1open = False
                        if self.servo2open == False:
                            self.move(self.pin_servo2,self.open2)
                            self.servo2open = True
                    elif self.RAW_CHAN1_RAW > 1700: ###OPEN SERVO1 AND CLOSE SERVO2
                        if self.servo1open == False:
                            self.move(self.pin_servo1,self.open1)
                            self.servo1open = True
                        if self.servo2open == True:
                            self.move(self.pin_servo2,self.close2)
                            self.servo2open = False
                    else:                           ####CLOSE BOTH SERVOS
                        if self.servo1open == True:
                            self.move(self.pin_servo1,self.close1)
                            self.servo1open = False
                        if self.servo2open == True:
                            self.move(self.pin_servo2,self.close2)
                            self.servo2open = False


                    #CHASER 3 Y 4

                    if self.RAW_CHAN2_RAW < 1300: ###OPEN SERVO4 AND CLOSE SERVO3
                        if self.servo3open == True:
                            self.move(self.pin_servo3,self.close3)
                            self.servo3open = False
                        if self.servo4open == False:
                            self.move(self.pin_servo4,self.open4)
                            self.servo4open = True
                    elif self.RAW_CHAN2_RAW > 1700: ###OPEN SERVO3 AND CLOSE SERVO4
                        if self.servo3open == False:
                            self.move(self.pin_servo3,self.open3)
                            self.servo3open = True
                        if self.servo4open == True:
                            self.move(self.pin_servo4,self.close4)
                            self.servo4open = False
                    else:                           ###CLOSE BOTH SERVOS
                        if self.servo3open == True: 
                            self.move(self.pin_servo3,self.close3)
                            self.servo3open = False
                        if self.servo4open == True:
                            self.move(self.pin_servo4,self.close4)
                            self.servo4open = False

    def gimbal_control(self):
        self.pitch_control(self.pitch_horizontal)
        self.yaw_control(self.yaw_forward)

    def switcher(self, vid_mod):
        if vid_mod==0:
           #print("switching to video1")
           self.move(self.pin_switch, self.hd_cam)
        if vid_mod==1:
            #print("switching to video2")
           self.move(self.pin_switch, self.thermal_cam)
        if vid_mod==2:
            #print("switching to video3")
            self.move(self.pin_switch, self.gimb_cam)
    ## SIMULATION FUNCTION
    def sim(self):
        while(True):
            command = input("Insert (Pin;Val) or Command:")
            command = command.upper()
            try:
                pin, val = command.split(";")
                self.move(int(pin),int(val))

            except:
                if command =="SAFEON":
                    self.RAW_CHAN3_RAW = 1900
                    print("SAFETY ENVELOPE ON")
                if command =="SAFEOFF":
                    self.RAW_CHAN3_RAW = 1500
                    print("SAFETY ENVELOPE OFF")
                if command == "CLOSE1":
                    self.RAW_CHAN1_RAW = 1900
                    print("OPEN SERVO1 AND CLOSE SERVO2")
                if command == "OPEN1":
                    self.RAW_CHAN1_RAW = 800
                    print("OPEN SERVO2 AND CLOSE SERVO1")
                if command == "CLOSE2":
                    self.RAW_CHAN2_RAW = 1900
                    print("OPEN SERVO3 AND CLOSE SERVO4")
                if command == "OPEN2":
                    self.RAW_CHAN2_RAW = 800
                    print("OPEN SERVO4 AND CLOSE SERVO3")
                if command == "OPEN":
                    self.RAW_CHAN1_RAW = 1900
                    self.RAW_CHAN2_RAW = 1900
                    print("SERVOS OPEN COMMAND")
                if command == "CLOSE":
                    self.RAW_CHAN1_RAW = 800
                    self.RAW_CHAN2_RAW = 800
                    print("SERVOS CLOSE COMMAND")
                if command =="GIMB_MAX":
                    self.pitch_control(1800)
                    self.yaw_control(1800)
                if command =="GIMB_MIN":
                    self.pitch_control(800)
                    self.yaw_control(800)
                if command =="VIDTEST":
                    val = 1000
                    while (val<1201):
                        self.move(self.pin_switch, val)
                        print(val)
                        val = val + 1
                if command =="HELP":
                    print("COMMANDS ARE: SAFEON, SAFEOFF, CLOSE1, CLOSE2, OPEN1, OPEN2, CLOSE, OPEN, VIDEO1, VIDEO2, VIDEO3")
                if command == "QUIT":
                    sys.exit()

                if (command =="VIDEO1"):
                    self.VIDEO_SWITCHER_SWC = 0
                    print("VIDEO SOURCE 1")
                if (command == "VIDEO2"):
                    self.VIDEO_SWITCHER_SWC = 1
                    print("VIDEO SOURCE 2")
                if (command == "VIDEO3"):
                    self.VIDEO_SWITCHER_SWC = 2
                    print("VIDEO SOURCE 3")

            #if (True):
            #    self.switcher(self.VIDEO_SWITCHER_SWC)
            #    self.control = self.VIDEO_SWITCHER_SWC

#START OF FUNCTION

raspi = Raspi()

#bat_sub = rospy.Subscriber("/mavros/battery",BatteryState, raspi.battery_callback)
if not rc_sim:
    rospy.init_node('mavros_pool',anonymous=True)
    position_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, raspi.position_callback)
    navsat_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, raspi.navsat_callback)
    hdg_sub = rospy.Subscriber("/mavros/global_position/compass_hdg",Float64, raspi.compass_hdg_callback)
    rc_sub = rospy.Subscriber("/mavros/rc/in",RCIn, raspi.rc_callback)
    rcout_sub = rospy.Subscriber("/mavros/rc/out",RCOut, raspi.rcout_callback)
    rate = rospy.Rate(10)

    t1=threading.Thread(target=raspi.launcher)
    t1.start()
    t2=threading.Thread(target=raspi.gimbal_control)
    t2.start()
    rospy.spin()

else:
    t3=threading.Thread(target=raspi.sim)
    t3.start()
    t1=threading.Thread(target=raspi.launcher)
    t1.start()

