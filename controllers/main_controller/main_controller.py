"""main_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Lidar, Motor, Robot


MAX_SPEED = 6.4
CRUISING_SPEED = 3.0
FAR_OBSTACLE_THRESHOLD = 0.4
NEAR_OBSTACLE_THRESHOLD = 0.7
FAST_DECREASE_FACTOR = 0.9
SLOW_DECREASE_FACTOR = 0.5
UNUSED_POINT = 83
N_SECTOR = 5

def check_speed(speed):
    if speed > MAX_SPEED:
        speed = MAX_SPEED
    return speed

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
time_step = int(robot.getBasicTimeStep())

# get devices
urg04lx = robot.getDevice("Hokuyo URG-04LX-UG01")
left_wheel = robot.getDevice("wheel_left_joint")
right_wheel = robot.getDevice("wheel_right_joint")

# init urg04lx
urg04lx.enable(time_step)
urg04lx_width = urg04lx.getHorizontalResolution()  # 667 points

# defines sector range
sector_range = [0] * N_SECTOR
sector_size = int((urg04lx_width - 2.0 * UNUSED_POINT - 1.0) / N_SECTOR)
for i in range(N_SECTOR):
    sector_range[i] = UNUSED_POINT + (i + 1) * sector_size

# defines useful points (above 5.6m / 2.0 = 2.8m, points not used)
max_range = urg04lx.getMaxRange()
range_threshold = max_range / 2.0

# init motors
left_speed = 0.0
right_speed = 0.0
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))
left_wheel.setVelocity(left_speed)
right_wheel.setVelocity(right_speed)

# control loop
while robot.step(time_step) != -1:
    # get lidar values
    urg04lx_values = urg04lx.getRangeImage()

    left_obstacle = 0.0
    front_left_obstacle = 0.0
    front_obstacle = 0.0
    front_right_obstacle = 0.0
    right_obstacle = 0.0

    for i in range(UNUSED_POINT, urg04lx_width - UNUSED_POINT - 1):
        if urg04lx_values[i] < range_threshold:
            if i < sector_range[0]:  # [83-182]
                left_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[0] <= i < sector_range[1]:  # [183-282]
                front_left_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[1] <= i < sector_range[2]:  # [283-382]
                front_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[2] <= i < sector_range[3]:  # [383-482]
                front_right_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[3] <= i < sector_range[4] + 1:  # [483-583]
                right_obstacle += (1.0 - urg04lx_values[i] / max_range)

    left_obstacle /= sector_size
    front_left_obstacle /= sector_size
    front_obstacle /= sector_size
    front_right_obstacle /= sector_size
    right_obstacle /= sector_size

    if left_obstacle > right_obstacle and left_obstacle > NEAR_OBSTACLE_THRESHOLD:
        speed_factor = (1.0 - FAST_DECREASE_FACTOR * left_obstacle) * MAX_SPEED / left_obstacle
        left_speed = check_speed(speed_factor * left_obstacle)
        right_speed = check_speed(speed_factor * right_obstacle)

    elif front_left_obstacle > front_right_obstacle and front_left_obstacle > FAR_OBSTACLE_THRESHOLD:
        speed_factor = (1.0 - SLOW_DECREASE_FACTOR * front_left_obstacle) * MAX_SPEED / front_left_obstacle
        left_speed = check_speed(speed_factor * front_left_obstacle)
        right_speed = check_speed(speed_factor * front_right_obstacle)

    elif front_right_obstacle > front_left_obstacle and front_right_obstacle > FAR_OBSTACLE_THRESHOLD:
        speed_factor = (1.0 - SLOW_DECREASE_FACTOR * front_right_obstacle) * MAX_SPEED / front_right_obstacle
        left_speed = check_speed(speed_factor * front_left_obstacle)
        right_speed = check_speed(speed_factor * front_right_obstacle)

    elif right_obstacle > left_obstacle and right_obstacle > NEAR_OBSTACLE_THRESHOLD:
        speed_factor = (1.0 - FAST_DECREASE_FACTOR * right_obstacle) * MAX_SPEED / right_obstacle
        left_speed = check_speed(speed_factor * left_obstacle)
        right_speed = check_speed(speed_factor * right_obstacle)

    elif front_obstacle > NEAR_OBSTACLE_THRESHOLD:
        speed_factor = (1.0 - FAST_DECREASE_FACTOR * front_obstacle) * MAX_SPEED / front_obstacle
        if front_right_obstacle > front_left_obstacle or right_obstacle > left_obstacle:
            left_speed = -check_speed(speed_factor * front_obstacle)
            right_speed = check_speed(speed_factor * front_obstacle)
        else:
            left_speed = check_speed(speed_factor * front_obstacle)
            right_speed = -check_speed(speed_factor * front_obstacle)
        time.sleep(0.001)

    else:
        left_speed = CRUISING_SPEED
        right_speed = CRUISING_SPEED

    # set actuators
    left_wheel.setVelocity(left_speed)
    right_wheel.setVelocity(right_speed)

# free(braitenberg_coefficients);
wb_robot_cleanup()

    
    

