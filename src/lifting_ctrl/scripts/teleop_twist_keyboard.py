#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from bt_task_msgs.msg import LiftMotorMsg
from bt_task_msgs.srv import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse

import sys, select, termios, tty

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

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.__publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        self.__lift_back_height = None
        self.__lift_target_height = None
        rospy.Subscriber('/LiftingMotorMsg', LiftMotorMsg, self.__lift_motor_callback)
        rospy.wait_for_message('/LiftingMotorMsg', LiftMotorMsg)
        self.__lift_done = True
        rospy.wait_for_service('/LiftingMotorService')
        self.__lift_motor_service = rospy.ServiceProxy('/LiftingMotorService', LiftMotorSrv)
        
        self.__x = 0.0
        self.__y = 0.0
        self.__z = 0.0
        self.__th = 0.0
        self.__speed = 0.0
        self.__turn = 0.0
        self.__lift_step = 0
        self.__condition = threading.Condition()
        self.__done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.__timeout = 1.0 / rate
        else:
            self.__timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.__publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.__publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(
        self,
        x:float,
        y:float,
        z:float,
        th:float,
        speed:float,
        turn:float,
        lift_step:float):
        self.__condition.acquire()
        self.__x = x
        self.__y = y
        self.__z = z
        self.__th = th
        self.__speed = speed
        self.__turn = turn
        self.__lift_step = lift_step
        # Notify publish thread that we have a new message.
        self.__condition.notify()
        self.__condition.release()

    def stop(self):
        self.__done = True
        self.update(0, 0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.__done:
            self.__condition.acquire()
            # Wait for a new message or timeout.
            self.__condition.wait(self.__timeout)

            # Copy state into twist message.
            twist.linear.x = self.__x * self.__speed
            twist.linear.y = self.__y * self.__speed
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.__th * self.__turn
            
            lift_motor_height = int(self.__lift_target_height + self.__z * self.__lift_step)
            
            self.__condition.release()

            # Publish.
            self.__publisher.publish(twist)
            
            # Lift motor control
            lift_motor_height = max(lift_range[0], min(lift_range[1], lift_motor_height))
            if lift_motor_height != self.__lift_target_height:
                req = LiftMotorSrvRequest()
                req.mode = 0
                req.val = lift_motor_height
                res:LiftMotorSrvResponse = self.__lift_motor_service(req)
                self.__lift_done = False
                print(f'Lift motor move from {self.__lift_target_height} mm to {lift_motor_height} mm')

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.__publisher.publish(twist)
        
    def __lift_motor_callback(self, msg:LiftMotorMsg):
        self.__lift_back_height = msg.backHeight
        self.__lift_target_height = msg.targetHeight
        if (not self.__lift_done) and (self.__lift_back_height == self.__lift_target_height):
            print(f'Lift motor has reached target height: {self.__lift_target_height}')
            self.__lift_done = True

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed:float, turn:float, lift_step:float):
    return f'currently:\tspeed {speed}\tturn {turn}\tlift step {lift_step}'

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    lift_step = rospy.get_param("~lift_step", (lift_range[1] - lift_range[0]) / 2 ** 3)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn, lift_step)

        print(msg)
        print(vels(speed,turn,lift_step))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn,lift_step))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in stepBindings.keys():
                lift_step_ = lift_step * stepBindings[key][0]
                if (lift_step_ >= 1) and (lift_step_ <= (lift_range[1] - lift_range[0])):
                    lift_step = lift_step_
                    print(vels(speed,turn,lift_step))
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, speed, turn, lift_step)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
