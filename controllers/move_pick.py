from controller import Robot
from controller import Motor
from controller import GPS
from controller import Compass
from controller import Camera
import numpy as np
# create the Robot instance.
class YouBotBase():
    def __init__(self) -> None:
        ## Parameters
        self.SPEED=4
        self.MAX_SPEED=0.30
        self.SPEED_INCREMENT=0.05
        self.DISTANCE_TOLERANCE=0.001
        self.ANGLE_TOLERANCE=0.001
        self.WHEEL_RADIUS=0.05
        self.LX=0.228
        self.LY=0.158
        self.TIME_STEP=16
        self.robot=Robot()
        # Get wheels
        self.wheels=[]
        for i in range(4):
            self.wheels.append(self.robot.getDevice("wheel"+str(i+1)))
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(0.0)
        
        # Get arm
        self.arms={}
        for i in range(5):
            self.arms[i+1]=(self.robot.getDevice("arm"+str(i+1)))
        self.arm_length={1:0.253,2:0.155,3:0.135,4:0.081,5:0.105}
        
        # Get camera
        self.camera=self.robot.getDevice("camera")
        self.camera.enable(self.TIME_STEP)
        # Get Sensor
        # self.gps=self.robot.getDevice("gps")
        # self.compass=self.robot.getDevice("compass")
    def passive_wait(self,seconds):
        """ passive wait for seconds

        Args:
            seconds (_type_): seconds to wait
        """
        start_time=self.robot.getTime()
        while self.robot.getTime()-start_time<seconds:
            self.step()
    def step(self):
        if(self.robot.step(self.TIME_STEP)==-1):
            quit()
    def arm_operate(self):
        # self.arms[2].setPosition(0.678)
        # self.arms[3].setPosition(0.682)
        # self.arms[4].setPosition(1.74)
        # self.arms[5].setPosition(0.0)
        self.arm_ik(0.1,0.15,0.23)
        pass
    
    def run(self):
        """ run the robot
        """
        count=0
        # while True:
        #     self.step()
        #     self.move_left()
        self.arm_operate() ## First operation
        self.passive_wait(100)
        self.move_right()
        self.passive_wait(5)
        self.stop()
        self.passive_wait(1)
        self.arm_operate() ## Second operation
        
        self.move_right()
        self.passive_wait(5)
        self.stop()
        self.passive_wait(1)
        self.arm_operate() ## Third operation
        while True:
            self.step()
    def set_wheel_velocity(self,velocity):
        """ set wheel velocity by a list of 4 velocity
        Args:
            velocity (_type_): list of 4 velocity
        """
        for i in range(4):
            self.wheels[i].setVelocity(velocity[i])
    def stop(self):
        """ stop
        """
        self.set_wheel_velocity([0]*4)
    def move_forward(self):
        """ move forward
        """
        self.set_wheel_velocity([self.SPEED]*4)
    def move_backward(self):
        """ move backward
        """
        self.set_wheel_velocity([-self.SPEED]*4)
    def move_left(self):
        """ move left
        """
        self.set_wheel_velocity([self.SPEED,-self.SPEED,-self.SPEED,self.SPEED])
    def move_right(self):
        """ move right
        """
        self.set_wheel_velocity([-self.SPEED,self.SPEED,self.SPEED,-self.SPEED])
    def turn_left(self):
        """ turn left
        """
        self.set_wheel_velocity([-self.SPEED,self.SPEED,-self.SPEED,self.SPEED])
    def turn_right(self):
        """ turn right
        """
        self.set_wheel_velocity([self.SPEED,-self.SPEED,self.SPEED,-self.SPEED])
    def arm_ik(self,x,y,z):
        """ inverse kinematics for arm
        Args:
            x (_type_): x coordinate
            y (_type_): y coordinate
            z (_type_): z coordinate
        """
        y1=np.sqrt(x**2+y**2)
        z1=z+self.arm_length[4]+self.arm_length[5]-self.arm_length[1]
        
        a=self.arm_length[2]
        b=self.arm_length[3]
        c=np.sqrt(y1**2+z1**2)
        
        alpha=-np.arcsin(x/y1)
        beta=-(np.pi/2-np.arccos((a**2+c**2-b**2)/(2.0*a*c))-np.arctan(z1/y1))
        gamma=-(np.pi-np.arccos((a**2+b**2-c**2)/(2.0*a*b)))
        delta=-(np.pi+beta+gamma)
        epsilon=np.pi/2+alpha
        
        ## Print all the data
        print(f'y1,z1,a,b,c: {y1},{z1},{a},{b},{c}')
        print(f'alpha,beta,gamma,delta,epsilon: {alpha},{beta},{gamma},{delta},{epsilon}')
        
        arm_setting=[alpha,beta,gamma,delta,epsilon]
        for i in range(5):
            self.arms[i+1].setPosition(arm_setting[i])
if __name__ =='__main__':
    robot=YouBotBase()
    robot.run()