from controller import Robot, Camera

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get the camera device and enable it.
CAM = robot.getDevice('camera')
CAM.enable(timestep)
CAM.recognitionEnable(timestep)

# Get the motor devices for the head joints.
head_1_joint = robot.getDevice('head_1_joint')

# Set the motor's velocity to achieve free movement.
max_velocity = head_1_joint.getMaxVelocity()
head_1_joint.setVelocity(max_velocity / 2.0)

# Get the motor devices for turning the robot.
wheel_left_joint = robot.getDevice('wheel_left_joint')
wheel_right_joint = robot.getDevice('wheel_right_joint')

# Set the wheel motors' velocity.
wheel_max_velocity = 3.0
wheel_left_joint.setVelocity(wheel_max_velocity)
wheel_right_joint.setVelocity(wheel_max_velocity)

# Initialize the target position for the head joint.
target_position_2 = 0.0
head_direction = 1

# Flag to track if an object is detected.
object_detected = False

# Flag to track if the robot is tracking the object.
tracking_object = False

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    recognized_object_array = CAM.getRecognitionObjects()

    if len(recognized_object_array) > 0:
        obj = recognized_object_array[0]
        image_position = obj.position_on_image
        print(image_position[0], image_position[1])

        image_width = CAM.getWidth()
        image_height = CAM.getHeight()
        print(image_width)
        print(image_height)

        if not object_detected:
            # Object is detected, position it at the center of the image.
            target_position_2 = 0.0
            object_detected = True

        if abs(image_position[0] - image_width / 2) < 40 and abs(image_position[1] - image_height / 2) < 40:
            # Object is at the center, stop the head movement.
            target_position_2 = 0.0

            if not tracking_object:
                # Start tracking the object.
                tracking_object = True

        if tracking_object:
            # Turn the robot towards the object.
            if image_position[0] < image_width / 2:
                wheel_left_joint.setVelocity(wheel_max_velocity)
                wheel_right_joint.setVelocity(-wheel_max_velocity)
            else:
                wheel_left_joint.setVelocity(-wheel_max_velocity)
                wheel_right_joint.setVelocity(wheel_max_velocity)
        else:
            # Move the head continuously to find the object.
            target_position_2 += 0.1 * head_direction
            head_1_joint.setPosition(target_position_2)

            # Reverse the head movement direction when reaching the limits
            if abs(target_position_2) >= 1.0:
                head_direction *= -1

        print("Object ID:", obj.id)
        print("Position:", list(obj.position))
        print("Orientation:", list(obj.orientation))
        print("Size:", list(obj.size))
        print("Position on Image:", list(obj.position_on_image))
        print("Size on Image:", list(obj.size_on_image))
        print("Number of Colors:", obj.number_of_colors)
        print("Colors:", list(obj.colors[:obj.number_of_colors]))
        print("Model:", obj.model)
        print()
    else:
        # No object is detected, stop tracking and move the head continuously.
        tracking_object = False
        target_position_2 += 0.1 * head_direction
        head_1_joint.setPosition(target_position_2)

        # Reverse the head movement direction when reaching the limits
        if abs(target_position_2) >= 1.0:
            head_direction *= -1

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
