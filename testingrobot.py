from hub import port
import motor_pair
import runloop
import math
degrees=360
cm = 7*2.54
degrees_percm = degrees/cm
slipfactor = 1.3
distanceslip = 67
slip = slipfactor/distanceslip
slipercent = slip/100




async def main():
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
    #for i in range(4):
    await drivecm(67)


async def drivecm(distance_cm):
    distance_degrees = int(distance_cm * degrees_percm)
    distance_degrees = int(distance_degrees * (1 + slipercent))
    print(f"Moving {distance_cm} cm, which is {distance_degrees} degrees")
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, distance_degrees, 0)
    #await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees, 0)



runloop.run(main())
    