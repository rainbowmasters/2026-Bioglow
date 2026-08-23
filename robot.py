# LEGO slot:0 autostart

import math
import motor
import motor_pair
import runloop
from hub import motion_sensor, port


class Robot:
    def __init__(self, left_port, right_port, wheel_diameter_cm=6):
        self._left_port = left_port
        self._right_port = right_port
        motor_pair.pair(motor_pair.PAIR_1, left_port, right_port)
        self._wheel_circumference_cm = wheel_diameter_cm * math.pi
        self._raw_yaw = 0.0
        self._yaw = 0.0
        self._running = True

    async def start(self):
        motion_sensor.reset_yaw(0)
        await runloop.until(motion_sensor.stable)

    @property
    def yaw(self):
        return self._yaw

    async def _yaw_monitor(self, interval_ms=10):
        while self._running:
            raw = motion_sensor.tilt_angles()[0] * -0.1
            delta = raw - self._raw_yaw
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            self._yaw += delta
            self._raw_yaw = raw
            await runloop.sleep_ms(interval_ms)

    def stop_monitoring(self):
        self._running = False

    # ---- movement ----
    
    @staticmethod
    def _trapezoidal_speed(progress, total, max_speed, min_speed, ramp):
        """Returns the speed for a point `progress` units into a move of
        length `total`, ramping up over the first `ramp` units and back
        down over the last `ramp` units. Falls back to a triangular
        profile (no flat cruise) if the move is shorter than 2*ramp."""
        if total <= 0:
            return max_speed
        ramp = min(ramp, total / 2)
        if ramp <= 0:
            return max_speed
        remaining = total - progress
        if progress < ramp:
            fraction = progress / ramp
        elif remaining < ramp:
            fraction = remaining / ramp
        else:
            fraction = 1.0
        fraction = max(0.0, min(1.0, fraction))
        return min_speed + (max_speed - min_speed) * fraction

    async def drive_straight(self, distance_cm, max_speed=300, min_speed=80,
                              ramp_cm=15, kp=6, tolerance_cm=0.3):
        target_heading = self._yaw
        direction = 1 if distance_cm >= 0 else -1
        total_cm = abs(distance_cm)
        target_degrees = (total_cm / self._wheel_circumference_cm) * 360

        motor.reset_relative_position(self._left_port, 0)

        while True:
            traveled_degrees = abs(motor.relative_position(self._left_port))
            if traveled_degrees >= target_degrees:
                break

            traveled_cm = (traveled_degrees / 360) * self._wheel_circumference_cm
            speed = self._trapezoidal_speed(traveled_cm, total_cm, max_speed, min_speed, ramp_cm)

            error = self._yaw - target_heading
            steering = max(-100, min(100, int(-error * kp)))
            motor_pair.move(motor_pair.PAIR_1, steering, velocity=direction * int(speed))
            await runloop.sleep_ms(10)

        motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)

    async def turn_to(self, target_heading, max_speed=300, min_speed=60,
                       ramp_deg=30, tolerance_deg=1.0, settle_ms=100):
        start_heading = self._yaw
        total_deg = abs(target_heading - start_heading)
        if total_deg <= tolerance_deg:
            return

        while True:
            error = target_heading - self._yaw
            if abs(error) <= tolerance_deg:
                break

            progress_deg = total_deg - abs(error)
            speed = self._trapezoidal_speed(progress_deg, total_deg, max_speed, min_speed, ramp_deg)
            direction = 1 if error > 0 else -1
            motor_pair.move_tank(
                motor_pair.PAIR_1,
                int(direction * speed),
                int(-direction * speed),
            )
            await runloop.sleep_ms(10)

        motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
        await runloop.sleep_ms(settle_ms)

    async def run(self):
        await self.drive_straight(150)

async def main():
    robot = Robot(port.B, port.A)
    await robot.start()
    await runloop.run(
        robot._yaw_monitor(),
        robot.run(),
    )
    robot.stop_monitoring()

runloop.run(main())