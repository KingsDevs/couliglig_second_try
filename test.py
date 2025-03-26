from controller import Robot, PositionSensor, InertialUnit

robot = Robot()
timestep = int(robot.getBasicTimeStep())

right_motor = robot.getDevice('right_motor')
left_motor = robot.getDevice('left_motor')

right_motor_sensor: PositionSensor = robot.getDevice('right_position_sensor')
left_motor_sensor: PositionSensor = robot.getDevice('left_position_sensor')

left_motor_sensor.enable(timestep)
right_motor_sensor.enable(timestep)

imu: InertialUnit = robot.getDevice('inertial unit')
imu.enable(timestep)

while robot.step(timestep) != -1:
    print(f"{imu.getRollPitchYaw()}")
    