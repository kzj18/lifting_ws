#!/usr/bin/env /home/kzj18/miniconda3/envs/NExplore_ros/bin/python3

import threading

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from bt_task_msgs.msg import LiftMotorMsg
from bt_task_msgs.srv import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
r/v : increase lift motor step by 100%/decrease lift motor step by 50%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

stepBindings={
        'r':(2,2),
        'v':(.5,.5),
    }

lift_range = (0, 700)

def vels(speed:float, turn:float, lift_step:float):
    return f'currently:\tspeed {speed}\tturn {turn}\tlift step {lift_step}'

class CV2Window:
    
    def __init__(self, frame_topic:str):
        
        rospy.init_node('cv2_window', anonymous=True)
        
        lift_step = rospy.get_param("~lift_step", (lift_range[1] - lift_range[0]) / 2 ** 3)
        speed = rospy.get_param("~speed", 0.1)
        turn = rospy.get_param("~turn", 0.2)
        repeat = rospy.get_param("~repeat_rate", 0.0)
        
        self.__speed = 0.0
        self.__turn = 0.0
        self.__lift_step = 0
        
        self.__frame = None
        self.__lift_back_height = None
        self.__lift_target_height = None
        rospy.Subscriber(frame_topic, Image, self.__frame_callback)
        rospy.wait_for_message(frame_topic, Image)
        self.__publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        self.__lift_done = False
        rospy.Subscriber('/lifter_1/LiftMotorStatePub', LiftMotorMsg, self.__lift_motor_callback)
        rospy.wait_for_message('/lifter_1/LiftMotorStatePub', LiftMotorMsg)
        self.__lift_done = True
        rospy.wait_for_service('/lifter_1/LiftingMotorService')
        self.__lift_motor_service = rospy.ServiceProxy('/lifter_1/LiftingMotorService', LiftMotorSrv)
        
        self.__done = False
        self.__condition = threading.Condition()
        self.__publish_thread = threading.Thread(target=self.__publish_thread_main, args=(repeat,))
        self.__publish_thread.start()
        
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

        self.__status = 0
        self.__x = 0.0
        self.__y = 0.0
        self.__z = 0.0
        self.__th = 0.0
        print(msg)
        print(vels(speed,turn,lift_step))
        
        while not rospy.is_shutdown():
            cv2.imshow('frame', self.__frame)
            key = cv2.waitKey(1)
            key_pressed = chr(key & 0xFF)
            x = 0
            y = 0
            z = 0
            th = 0
            if key in [27]:
                break
            elif key_pressed in moveBindings.keys():
                x = moveBindings[key_pressed][0]
                y = moveBindings[key_pressed][1]
                z = moveBindings[key_pressed][2]
                th = moveBindings[key_pressed][3]
            elif key_pressed in speedBindings.keys():
                speed = speed * speedBindings[key_pressed][0]
                turn = turn * speedBindings[key_pressed][1]

                print(vels(speed,turn,lift_step))
                self.__update_status()
            elif key_pressed in stepBindings.keys():
                lift_step_ = lift_step * stepBindings[key_pressed][0]
                if (lift_step_ >= 1) and (lift_step_ <= (lift_range[1] - lift_range[0])):
                    lift_step = lift_step_
                    print(vels(speed,turn,lift_step))
                    self.__update_status()
            else:
                continue
                    
            with self.__condition:
                self.__x = x
                self.__y = y
                self.__z = z
                self.__th = th
                self.__speed = speed
                self.__turn = turn
                self.__lift_step = lift_step
                self.__condition.notify_all()
            
        cv2.destroyAllWindows()
        self.__done = True
        with self.__condition:
            self.__x = 0.0
            self.__y = 0.0
            self.__z = 0.0
            self.__th = 0.0
            self.__speed = 0.0
            self.__turn = 0.0
            self.__lift_step = 0
            self.__condition.notify_all()
        self.__publish_thread.join()
        
    def __publish_thread_main(self, rate):
        twist = Twist()
        
        if rate != 0.0:
            timeout = 1.0 / rate
        else:
            timeout = None
        
        while not self.__done:
            with self.__condition:
                # Wait for a new message or timeout.
                self.__condition.wait(timeout)

                # Copy state into twist message.
                twist.linear.x = self.__x * self.__speed
                twist.linear.y = self.__y * self.__speed
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.__th * self.__turn
                
                lift_motor_height = int(self.__lift_target_height + self.__z * self.__lift_step)

            # Publish.
            self.__publisher.publish(twist)
            
            # Lift motor control
            lift_motor_height = max(lift_range[0], min(lift_range[1], lift_motor_height))
            if lift_motor_height != self.__lift_target_height:
                if self.__lift_back_height < 10:
                    req = LiftMotorSrvRequest()
                    req.mode = -4
                    req.val = 100
                    res:LiftMotorSrvResponse = self.__lift_motor_service(req)
                req = LiftMotorSrvRequest()
                req.mode = 0
                req.val = lift_motor_height
                print(f'Lift motor move from {self.__lift_target_height} mm to {lift_motor_height} mm')
                res:LiftMotorSrvResponse = self.__lift_motor_service(req)
                self.__lift_done = False

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.__publisher.publish(twist)
        
    def __frame_callback(self, image: Image):
        frame = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.__lift_back_height is not None and self.__lift_target_height is not None:
            cv2.putText(frame, f"Back Height: {self.__lift_back_height}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Target Height: {self.__lift_target_height}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        if self.__speed is not None and self.__turn is not None and self.__lift_step is not None:
            cv2.putText(frame, vels(self.__speed, self.__turn, self.__lift_step).replace('\t', ' '), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        self.__frame = frame
        
    def __lift_motor_callback(self, msg:LiftMotorMsg):
        self.__lift_back_height = msg.backHeight
        self.__lift_target_height = msg.targetHeight
        if (not self.__lift_done) and (self.__lift_back_height == self.__lift_target_height):
            self.__lift_done = True
            
    def __update_status(self):
        if self.__status == 14:
            print(msg)
        self.__status = (self.__status + 1) % 15

if __name__ == '__main__':
    frame_topic = rospy.get_param('~frame_topic', '/camera/color/image_raw')
    CV2Window(frame_topic)