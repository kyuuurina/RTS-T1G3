from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Get the motor devices for the gripper finger joints.
gripper_left_finger_joint = robot.getDevice('gripper_left_finger_joint')
gripper_right_finger_joint = robot.getDevice('gripper_right_finger_joint')

# Set the motor's velocity to achieve free movement.
max_velocity = gripper_left_finger_joint.getMaxVelocity()
gripper_left_finger_joint.setVelocity(max_velocity / 2.0)

# Get the motor devices for the arm joints.
arm_joint_motors = []
for i in range(1, 8):
    motor = robot.getDevice('arm_' + str(i) + '_joint')
    arm_joint_motors.append(motor)

    # Set the maximum velocity for the arm joint motor.
    arm_max_velocity = motor.getMaxVelocity()
    motor.setVelocity(arm_max_velocity / 2.0)

# Initialize the gripper state
is_grasping = False

# Function to control the gripper
def control_gripper(grasp):
    if grasp:
        # Close the gripper
        gripper_left_finger_joint.setPosition(0.02)
        gripper_right_finger_joint.setPosition(0.02)
    else:
        # Open the gripper
        gripper_left_finger_joint.setPosition(0.0)
        gripper_right_finger_joint.setPosition(0.0)

# Function to control the arm joints
def control_arm_joints(angles):
    for i in range(len(angles)):
        arm_joint_motors[i].setPosition(angles[i])

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Example: Press 'G' to toggle the gripper state
    key = keyboard.getKey()
    if (key == ord('G') or key == ord('g')):
        print("G")
        is_grasping = not is_grasping
        control_gripper(is_grasping)

    # Example: Press 'M' to move the arm to a predefined position
    if (key == ord('M') or key == ord('m')):
        # Set the desired joint angles
        print("M")
        desired_joint_angles = [1.57, 1.0, -0.5, 1.0, 1.0, 0.5, 0.0]
        control_arm_joints(desired_joint_angles)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.