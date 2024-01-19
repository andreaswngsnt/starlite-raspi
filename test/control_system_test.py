from control_system import ControlSystem

control_system = ControlSystem()
control_system.activate()

while True:
    target_distance = float(input("Enter distance to travel: "))
    if target_distance < 0:
        continue

    target_angle = float(input("Enter angle to face: "))
    if target_angle > 180 or target_angle <= -180:
        continue

    control_system.rotate(target_angle)
    print("")
    print("")
    control_system.move(target_distance)
    print("")
    print("")