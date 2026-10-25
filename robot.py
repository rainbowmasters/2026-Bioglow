# LEGO slot:19 autostart
import math
import motor
import motor_pair
import runloop
import color
import color_sensor
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

            await runloop.sleep_ms(10)

        motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
        left_minus_right = (abs(motor.relative_position(self._left_port))
                            - abs(motor.relative_position(self._right_port)))
        print(f"drive done: heading at stop = {self.heading():.1f} L-R={left_minus_right}")

    async def turn(self, degrees, speed=200, kp=5.0, min_speed=100,
                   tolerance=0.3, timeout_ms=4000, stop_action=motor.HOLD,
                   lead_coeff=0.0175):
        """Spins in place by `degrees` -- positive = clockwise (right),
        negative = counter-clockwise (left), matching heading(). Uses the gyro
        rather than wheel rotation so wheel slip doesn't matter. Wheel speed is
        proportional to the remaining angle, capped at `speed` and floored at
        `min_speed` so the robot doesn't stall just short of the target.

        `min_speed` is a floor, not a target, and it is deliberately high.
        Below about 100 the rear wheels' friction stalls the robot outright a
        couple of degrees short -- the trace shows the error frozen and
        rate=0 while the wheels are still commanded -- and it then breaks away
        and lurches a good two degrees past. Crawling in slowly is the one
        thing that does not work here.

        Moving at a speed it can actually sustain, the robot carries a
        repeatable distance past the point where the loop decides to stop, so
        the loop stops early by `lead_coeff` degrees per deg/s of wheel speed
        instead of at a fixed band. `lead_coeff` is the number to recalibrate:
        it's the `coast=` figure printed at the end divided by the wheel speed
        the loop exited at, measured here at 1.75 degrees of carry at 100
        deg/s. It shifts with the robot's weight and the mat.

        One consequence of leading the stop: a turn asking for less than
        about (tolerance + min_speed * lead_coeff) degrees, so ~2 here, is a
        no-op. Small corrections are also where the model is weakest, since
        the robot never reaches a steady rate. Gives up after `timeout_ms` so
        a blocked robot can't hang the run."""
        target_yaw = degrees
        max_speed = min(abs(int(speed)), 1000)
        loop_count = 0
        # Ticks rather than a clock, so there's no dependency on time.ticks_ms
        # and the bound moves with the sleep below if that ever changes.
        tick_ms = 10
        max_loops = max(1, int(timeout_ms) // tick_ms)

        while True:
            # Remaining angle, wrapped so a target past +/-180 doesn't
            # produce a 360-degree "error". Positive = still need to go
            # clockwise.
            error = ((target_yaw - self.heading() + 180) % 360) - 180
            wheel_speed = max(min_speed, min(max_speed, int(kp * abs(error))))

            # How far the robot will carry past the target if we stop now.
            # Worked out before the exit test rather than after, because the
            # speed we're about to command is what sets the carry. Once
            # kp*error drops under min_speed the speed stops falling, so
            # without this the last stretch runs at a fixed rate into a fixed
            # band and the carry lands entirely in the final error.
            lead = wheel_speed * lead_coeff
            if abs(error) <= tolerance + lead:
                exit_lead = lead
                break
            if loop_count >= max_loops:
                exit_lead = lead
                print(f"turn: TIMED OUT after {timeout_ms}ms, err={error:.1f}")
                break

            if error < 0:
                wheel_speed = -wheel_speed
            # Clockwise = left wheel forward, right wheel backward.
            motor_pair.move_tank(motor_pair.PAIR_1, wheel_speed, -wheel_speed,
                                 acceleration=5000)

            # 10ms rather than 20ms: the loop can only stop within one tick
            # of crossing the band, so a shorter tick is a tighter landing.
            await runloop.sleep_ms(tick_ms)
            loop_count += 1

        # Sampled before the stop command, as close to the exit as possible,
        # so the difference against the settled heading below is the coast and
        # nothing else. Reading it after the stop call -- as this used to --
        # samples the robot mid-movement and reports the moment of the break
        # twice over, which hides the coast entirely.
        heading_at_brake = self.heading()
        motor_pair.stop(motor_pair.PAIR_1, stop=stop_action)

        # Let the chassis stop rocking before believing the gyro, otherwise
        # this reports a wobble rather than where the robot ended up.
        await runloop.sleep_ms(120)
        settled_heading = self.heading()
        error = ((target_yaw - settled_heading + 180) % 360) - 180
        coast = ((settled_heading - heading_at_brake + 180) % 360) - 180
        # coast/lead is the reality check on lead_coeff: at 1.0 the carry is
        # being predicted correctly, and coast/wheel_speed is the value to
        # retune lead_coeff to if it isn't.
        print(f"turn done: heading at stop = {settled_heading:.1f} err={error:.1f} coast={coast:.1f} lead={exit_lead:.1f}")

    async def program0(self):
        await self.drive(distance_cm=20, speed = 800)
        await self.turn(90, speed = 600)
        await runloop.sleep_ms(1000)
        await self.turn(180, speed = 600)
        await runloop.sleep_ms(1000)
        await self.turn(270, speed = 600)
        await runloop.sleep_ms(1000)
        await self.turn(360, speed = 600)
        await runloop.sleep_ms(1000)
        await self.drive(distance_cm=-18, speed = 800)

    async def program1(self):
        await self.drive(distance_cm=20, speed = 800)
        await self.turn(45, speed = 400)
        await self.drive(distance_cm=51, speed = 800)
        await self.turn(10, speed = 400)
        # await self.drive(distance_cm=-2, speed = 700)
        # await self.turn(3, speed = 600)
        await runloop.sleep_ms(1000)
        motor.run_for_degrees(port.A, 450, 800)
        await motor.run_for_degrees(port.B, -450, 800)
        await self.drive(distance_cm=-10, speed = 700)
        await self.turn(45, speed = 600)
        await runloop.sleep_ms(5)
        motor.run_for_degrees(port.A, -1700, 800)
        await motor.run_for_degrees(port.B, 1700, 800)

    async def program2(self):
        await self.rotate_accessory(port.D, degrees=300)

    async def program4(self):
        await self.drive(distance_cm=4)
        await self.turn(45)
        await self.drive(distance_cm=41)
        motor.run_for_degrees(port.D, 300, 360)
        await self.drive(distance_cm=-5, speed=300)
        await self.drive(distance_cm=-35, speed=700)
        await self.turn(45)
        await self.drive(distance_cm=-4, speed=700)

    async def program5(self):
        await self.drive(distance_cm=61, speed = 1000)
        # await self.turn(-52, speed = 600)
        # await self.drive(distance_cm=1, speed = 400)
        # await self.drive(distance_cm=-1, speed = 400)
        #await self.turn(52, speed = 450)
        #await self.motorturn(degrees = 300, velocity = 420)
        motor.run_for_degrees(port.A, 300, 520)
        await motor.run_for_degrees(port.B, 300, 520)
        # await self.drive(distance_cm=-3, speed = 400)
        # await motor.run_for_degrees(port.A, 100, 420)
        # await self.drive(distance_cm=1, speed = 400)
        await runloop.sleep_ms(1)
        motor.run_for_degrees(port.A, -450, 420)
        await motor.run_for_degrees(port.B, -450, 420)
        await self.drive(distance_cm=-70, speed = 1000)

    async def program6(self):
        # await self.drive(distance_cm=20, speed = 400)
        # await self.turn(45, speed = 600)
        # await self.drive(distance_cm=68, speed = 400)
        # await self.turn(20, speed = 600)
        # await self.drive(distance_cm=27, speed = 400)
        # await self.turn(25, speed = 600)
        # await self.drive(distance_cm=33, speed = 400)
        # await self.turn(30, speed = 600)
        # await self.drive(distance_cm=19, speed = 400)
        # await self.turn(-70, speed = 600)
        # motor.run_for_degrees(port.A, 300, 3070)
        # await motor.run_for_degrees(port.B, -300, 300)
        await self.drive(distance_cm=10, speed = 400)
        await self.turn(77, speed = 600)
        await self.drive(distance_cm=96, speed = 400)
        await self.turn(33, speed = 600)
        await self.drive(distance_cm=60, speed = 400)
        await self.rotate_accessory(port.D, degrees=320)
        await self.drive(distance_cm=-7, speed = 400)
        await self.turn(-27, speed = 600)
        await self.drive(distance_cm=-77, speed = 400)

    async def program7(self):
        motor.run_for_degrees(port.A, 1200, 1000)
        await motor.run_for_degrees(port.B, -1200, 1000)
        await runloop.sleep_ms(10)
        await self.drive(distance_cm=-8, speed = 400)
        await runloop.sleep_ms(10)
        motor.run_for_degrees(port.A, 700, 1000)
        await motor.run_for_degrees(port.B, -700, 1000)
        await self.drive

    async def programTest(self):
        for degrees in (90, 45, 180, 360):
            print(f"--- {degrees} degrees ---")
            for _ in range(2):
                await self.turn(degrees)
                await runloop.sleep_ms(500)
                await self.turn(0)
                await runloop.sleep_ms(500)

async def main():
    print("starting...")
    robot = Robot(port.B, port.A)
    await robot.start()
    await robot.programTest()
    await robot.stop()

runloop.run(main())