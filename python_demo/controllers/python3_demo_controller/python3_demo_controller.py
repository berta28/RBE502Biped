"""python3_demo_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
TIME_STEP = 64
# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:


motor_l_leg = robot.getDevice("LegLowerL")
motor_l_leg.setPosition(-.45)

motor_l_leg = robot.getDevice("LegUpperL")
motor_l_leg.setPosition(.45)

motor_l_leg = robot.getDevice("FootL")
motor_l_leg.setPosition(.5)

motor_l_ankle = robot.getDevice("AnkleL")
motor_l_ankle.setPosition(.45)



motor_l_leg = robot.getDevice("LegLowerR")
motor_l_leg.setPosition(.45)

motor_l_leg = robot.getDevice("LegUpperR")
motor_l_leg.setPosition(.45)

motor_l_leg = robot.getDevice("FootL")
motor_l_leg.setPosition(.5)

motor_l_leg = robot.getDevice("AnkleR")
# motor_l_leg.setPosition(.45)
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
