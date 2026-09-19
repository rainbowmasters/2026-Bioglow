# LEGO slot:2 autostart

import math
import motor
import motor_pair
import runloop
from hub import motion_sensor, port

class Robot:
    def __init__(self, 
                 left_port, 
                 right_port, 
                 wheel_diameter_cm=6,
                 track_width_cm=12):
        self._left_port = left_port
        self._right_port = right_port
        motor_pair.pair(motor_pair.PAIR_1, left_port, right_port)
        self._wheel_circumference_cm = wheel_diameter_cm * math.pi
        self._track_width_cm = track_width_cm
        self._raw_yaw = 0.0
        self._yaw = 0.0
        self._running = True
        self._calibration = 0.0

    async def start(self):
        motion_sensor.reset_yaw(0)
        await runloop.until(motion_sensor.stable)

    async def stop(self):
        motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)

    # # ---- accessories ----

    async def rotate_accessory(self, accessory_port, degrees, speed=500,
                                stop=motor.BRAKE, acceleration=1000,
                                deceleration=1000):
        """Rotates the motor on `accessory_port` by `degrees` from wherever it
        is now -- positive one way, negative the other. Waits until the move
        has finished, or until the motor stalls so a jammed attachment cannot
        hang the run."""
        await motor.run_for_degrees(accessory_port, int(degrees),
                                    abs(int(speed)), stop=stop,
                                    acceleration=acceleration,
                                    deceleration=deceleration)

    async def rotate_accessories(self, *moves, speed=500, stop=motor.BRAKE,
                                  acceleration=1000, deceleration=1000):
        """Starts every move at once and waits for them all to finish -- each
        move is a (port, degrees) pair. The motors are started without being
        awaited rather than through a second runloop, so the yaw monitor in the
        outer loop keeps ticking while the accessories turn."""
        ports = [accessory_port for accessory_port, _ in moves]
        for accessory_port, degrees in moves:
            motor.run_for_degrees(accessory_port, int(degrees), abs(int(speed)),
                                  stop=stop, acceleration=acceleration,
                                  deceleration=deceleration)
        # velocity() also reads 0 before the motors have spun up, so give them
        # a moment before treating "stopped" as "finished".
        await runloop.sleep_ms(50)
        await runloop.until(lambda: all(motor.velocity(p) == 0 for p in ports))

    @staticmethod
    def _speed_ramp(progress_cm, total_cm, max_speed, start_ramp_cm=30, end_ramp_cm=30):
        fraction = 1.0
        if progress_cm < start_ramp_cm:
            fraction = (progress_cm / start_ramp_cm * 0.8) + 0.2
        elif progress_cm > total_cm - end_ramp_cm:
            fraction = ((total_cm - progress_cm) / end_ramp_cm * 0.8) + 0.2
        else:
            fraction = 1.0
        return int(float(max_speed) * fraction)

    def heading(self):
        """Yaw in degrees, clockwise positive. tilt_angles() reports
        decidegrees with the opposite sign, hence the -0.1."""
        return motion_sensor.tilt_angles()[0] * -0.1

    def log_heading(self):
        print(f"heading: {self.heading()}")

    async def drive(self, distance_cm=100, speed=700, kp=12.0):
        """Drives `distance_cm` (negative = backwards) while holding the heading
        it started on. `kp` is the wheel-speed difference in deg/s applied per
        degree of heading error. Keep `speed` under ~800 so the faster wheel
        still has headroom -- at the motor's limit the correction can't act."""
        target_yaw = self.heading()
        motor.reset_relative_position(self._left_port, 0)
        motor.reset_relative_position(self._right_port, 0)
        direction = 1 if distance_cm >= 0 else -1
        max_speed = min(abs(int(speed)), 1000)
        total_cm = abs(distance_cm)
        target_degrees = (total_cm / self._wheel_circumference_cm) * 360
        loop_count = 0

        while True:
            traveled_degrees = (abs(motor.relative_position(self._left_port))
                                + abs(motor.relative_position(self._right_port))) / 2
            if traveled_degrees >= target_degrees:
                break

            traveled_cm = (traveled_degrees / 360) * self._wheel_circumference_cm
            base_speed = self._speed_ramp(traveled_cm, total_cm, max_speed)
            # Clamp against the ramped speed, not the max, so a correction
            # during the slow start/end can never reverse a wheel.
            max_correction = base_speed // 2

            # Positive error = nose is right of the target heading. Wrap so a
            # heading near +/-180 doesn't produce a 360-degree "error".
            error = ((self.heading() - target_yaw + 180) % 360) - 180
            correction = max(-max_correction, min(max_correction, int(kp * error)))

            # Nose right -> slow the left wheel, speed up the right. The same
            # formula holds in reverse: more reverse on the left swings the
            # nose left.
            left_speed = direction * base_speed - correction
            right_speed = direction * base_speed + correction
            # The ramp above asks for >1000 deg/s^2 near the ends of the drive;
            # move_tank's default acceleration of 1000 would clip that and
            # swallow the left/right difference. Raise it so the commanded
            # speeds are actually tracked.
            motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed,
                                 acceleration=5000)

            # Periodic trace so a log shows *when* the heading moves.
            loop_count += 1
            if loop_count % 20 == 0:
                # abs() because the left motor is mounted mirrored and counts
                # negative going forward. Positive = left wheel has traveled further.
                left_minus_right = (abs(motor.relative_position(self._left_port))
                                    - abs(motor.relative_position(self._right_port)))
                print(f"drive: {traveled_cm:.1f}cm base={base_speed} err={error:.1f} corr={correction} L-R={left_minus_right}")

            await runloop.sleep_ms(10)

        motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
        left_minus_right = (abs(motor.relative_position(self._left_port))
                            - abs(motor.relative_position(self._right_port)))
        print(f"drive done: heading at stop = {self.heading():.1f} L-R={left_minus_right}")

    async def turn(self, degrees, speed=200, kp=6.0, min_speed=60,
                   tolerance=1.0):
        """Spins in place by `degrees` -- positive = clockwise (right),
        negative = counter-clockwise (left), matching heading(). Uses the gyro
        rather than wheel rotation so wheel slip doesn't matter. Wheel speed is
        proportional to the remaining angle, capped at `speed` and floored at
        `min_speed` so the robot doesn't stall just short of the target."""
        target_yaw = self.heading() + degrees
        max_speed = min(abs(int(speed)), 1000)
        loop_count = 0

        while True:
            # Remaining angle, wrapped so a target past +/-180 doesn't
            # produce a 360-degree "error". Positive = still need to go
            # clockwise.
            error = ((target_yaw - self.heading() + 180) % 360) - 180
            if abs(error) <= tolerance:
                break

            wheel_speed = max(min_speed, min(max_speed, int(kp * abs(error))))
            if error < 0:
                wheel_speed = -wheel_speed
            # Clockwise = left wheel forward, right wheel backward.
            motor_pair.move_tank(motor_pair.PAIR_1, wheel_speed, -wheel_speed,
                                 acceleration=5000)

            loop_count += 1
            if loop_count % 20 == 0:
                print(f"turn: err={error:.1f} speed={wheel_speed}")

            await runloop.sleep_ms(10)

        motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
        print(f"turn done: heading at stop = {self.heading():.1f} err={error:.1f}")

    async def program1(self):
        self.log_heading()
        await self.drive(distance_cm=42, speed=900)
        await self.drive(distance_cm=-30, speed=900)
        self.log_heading()

    async def program2(self):
        await self.rotate_accessory(port.C, degrees=-130)
        # forward 60cm
        await self.drive(distance_cm=72, speed=700)
        # sleep
        runloop.sleep_ms(200)
        # turn 45 left
        await self.turn(-42, speed=100)
        # forward 13cm
        await self.drive(distance_cm=14, speed=300)
        # accessory D 180
        await self.rotate_accessory(port.C, degrees=-120)

        await self.drive(distance_cm=-14,speed=400)

        await self.turn(42, speed=100)

        await self.drive(distance_cm=-72, speed=900)

async def main():
    print("starting...")
    robot = Robot(port.B, port.A)
    await robot.start()
    await robot.program2()
    await robot.stop()

runloop.run(main())
