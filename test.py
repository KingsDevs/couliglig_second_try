from controller import Robot, PositionSensor

robot = Robot()
timestep = int(robot.getBasicTimeStep())

right_motor = robot.getDevice('right_motor')
left_motor = robot.getDevice('left_motor')

right_motor_sensor: PositionSensor = robot.getDevice('right_position_sensor')
left_motor_sensor: PositionSensor = robot.getDevice('left_position_sensor')

left_motor_sensor.enable(timestep)
right_motor_sensor.enable(timestep)

while robot.step(timestep) != -1:
    print(f'Left motor sensor: {left_motor_sensor.getValue()}')
    print(f'Right motor sensor: {right_motor_sensor.getValue()}')