from controller import Robot
from controller import Motor
from controller import GPS
from controller import Compass

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
        self.arms=[]
        for i in range(5):
            self.arms.append(self.robot.getDevice("arm"+str(i+1)))
        
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
        pass
    
    def run(self):
        """ run the robot
        """
        count=0
        # while True:
        #     self.step()
        #     self.move_left()
        self.arm_operate() ## First operation
        
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


if __name__ =='__main__':
    robot=YouBotBase()
    robot.run()