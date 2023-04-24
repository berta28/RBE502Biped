"""python3_demo_controller controller."""
#https://www.cyberbotics.com/doc/reference/motor?tab-language=python
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
TIME_STEP = 64
# create the Robot instance.
def main():
    robot = Robot()
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    # You should insert a getDevice-like function in order to get the
    # motor_l_leg.setPosition(.45)
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    # motor_l_leg = robot.getDevice("FootL")
    # Main loop:
    angle =0
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        pos_current = getcurrentpositions(robot)
        desired_pos = getdesired()
        setpositions(robot,desired_pos)
        
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass
# Enter here exit cleanup code.

def getcurrentpositions(robot):
    sampling_period = 64
    robot_leg_right_upper = robot.getDevice("LegUpperRS")
    robot_leg_right_upper.enable(sampling_period)
    robot_leg_right_lower = robot.getDevice("LegLowerRS")
    robot_leg_right_lower.enable(sampling_period)
    robot_leg_right_ankle = robot.getDevice("AnkleRS")
    robot_leg_right_ankle.enable(sampling_period)
    
    robot_leg_left_upper = robot.getDevice("LegUpperLS")
    robot_leg_left_upper.enable(sampling_period)
    robot_leg_left_lower =robot.getDevice("LegLowerLS")
    robot_leg_left_lower.enable(sampling_period)
    robot_leg_left_ankle = robot.getDevice("AnkleLS")
    robot_leg_left_ankle.enable(sampling_period)
    
    return [robot_leg_right_upper.getValue(),robot_leg_right_lower.getValue(),robot_leg_right_ankle.getValue(),robot_leg_left_upper.getValue(),robot_leg_left_lower.getValue(),robot_leg_left_ankle.getValue()]

def setpositions(robot,angles):
    robot_leg_right_upper = robot.getDevice("LegUpperR")
    robot_leg_right_upper.setPosition(angles[0])
    robot_leg_right_lower = robot.getDevice("LegLowerR")
    robot_leg_right_lower.setPosition(angles[1])
    robot_leg_right_ankle = robot.getDevice("AnkleR")
    robot_leg_right_ankle.setPosition(angles[2])
    
    robot_leg_left_upper = robot.getDevice("LegUpperL")
    robot_leg_left_upper.setPosition(angles[3])
    robot_leg_left_lower =robot.getDevice("LegLowerL")
    robot_leg_left_lower.setPosition(angles[4])
    robot_leg_left_ankle = robot.getDevice("AnkleL")
    robot_leg_left_ankle.setPosition(angles[5])
    return None

def getdesired():
    return [0,0,0,0,0,0]

if __name__ == '__main__':
    main()
