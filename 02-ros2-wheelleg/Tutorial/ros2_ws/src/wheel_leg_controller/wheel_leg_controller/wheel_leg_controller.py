import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from rm_msgs.msg import WheelLegCmd

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller


DT = 0.002
WHEEL_RADIUS = 0.06
JOINT_NAMES = (
    'right1', 'right2', 'left1', 'left2',
    'rightwheel', 'leftwheel'
)


def clamp(value, limit):
    return float(np.clip(value, -limit, limit))


def stamp_key(msg):
    stamp = msg.header.stamp
    return stamp.sec * 1000000000 + stamp.nanosec


def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w

    roll = math.atan2(
        2.0 * (w*x + y*z),
        1.0 - 2.0 * (x*x + y*y)
    )

    sin_pitch = 2.0 * (w*y - z*x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    yaw = math.atan2(
        2.0 * (w*z + x*y),
        1.0 - 2.0 * (y*y + z*z)
    )

    return roll, pitch, yaw


class LegLengthPID:
    def __init__(self):
        self.kp = 700.0
        self.ki = 10.0
        self.kd = 5.0

        self.dt = 0.002
        self.integral = 0.0
        self.last_error = 0.0

        self.integral_limit = 0.2
        self.output_limit = 150.0

    def compute(self, current, target):
        error = target - current

        # 积分项
        self.integral += error * self.dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral)
        )

        # 保留原有 PD 差分定义
        derivative = error - self.last_error
        self.last_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        return max(
            -self.output_limit,
            min(self.output_limit, output)
        )



class Stage3LegPID:
    def __init__(self, kp=700.0, ki=50.0, kd=0.0, dt=DT):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.last_error = None

    def compute(self, current, target):
        error = target - current
        derivative = (
            (error - self.last_error) / self.dt
            if self.last_error is not None else 0.0
        )

        candidate = float(np.clip(
            self.integral + error * self.dt,
            -0.2, 0.2
        ))

        raw = (
            self.kp * error
            + self.ki * candidate
            + self.kd * derivative
        )

        output = clamp(raw, 100.0)

        if abs(raw) <= 100.0 or error * raw < 0:
            self.integral = candidate

        self.last_error = error
        return output


class WheelSpeedPI:
    def __init__(
        self, kp=0.8, ki=1.5,
        max_torque=0.8, dt=DT
    ):
        self.kp = kp
        self.ki = ki
        self.max_torque = max_torque
        self.dt = dt
        self.integral = 0.0

    def compute(self, target, measured):
        error = target - measured

        candidate = self.integral + error * self.dt
        candidate = float(np.clip(candidate, -1.0, 1.0))

        raw_output = (
            self.kp * error + self.ki * candidate
        )

        output = float(np.clip(
            raw_output,
            -self.max_torque,
            self.max_torque
        ))

        # 抗积分饱和
        if abs(raw_output) <= self.max_torque:
            self.integral = candidate
        elif error * raw_output < 0:
            self.integral = candidate

        return output


class WheelLegController(Node):
    def __init__(self):
        super().__init__('wheel_leg_controller')

        self.declare_parameter('enable_control', False)
        self.declare_parameter('target_length', 0.10)
        self.declare_parameter('pitch_sign', -1.0)
        self.declare_parameter('gyro_pitch_sign', -1.0)
        self.declare_parameter('leg_length_enable', False)
        self.declare_parameter('wheel_pi_enable', False)
        self.declare_parameter('wheel_kp', 1.2)
        self.declare_parameter('wheel_ki', 2.0)
        self.declare_parameter('wheel_max_torque', 1.2)


        self.enable_control = bool(
            self.get_parameter('enable_control').value
        )
        self.target_length = float(
            self.get_parameter('target_length').value
        )
        self.pitch_sign = float(
            self.get_parameter('pitch_sign').value
        )
        self.gyro_pitch_sign = float(
            self.get_parameter('gyro_pitch_sign').value
        )

        self.wheel_pi_enable = bool(
            self.get_parameter('wheel_pi_enable').value
        )
        self.wheel_kp = float(
            self.get_parameter('wheel_kp').value
        )
        self.wheel_ki = float(
            self.get_parameter('wheel_ki').value
        )
        self.wheel_max_torque = float(
            self.get_parameter('wheel_max_torque').value
        )

        if self.wheel_max_torque <= 0:
            raise ValueError('wheel_max_torque must be positive')

        self.right_links = five_links_param(DT)
        self.left_links = five_links_param(DT)

        self.right_lqr = lqr_controller(DT)
        self.left_lqr = lqr_controller(DT)

        self.leg_length_enable = bool(
            self.get_parameter('leg_length_enable').value
        )
        if self.leg_length_enable and self.wheel_pi_enable:
            raise ValueError('Stage 2 and Stage 3 cannot run together')
        self.stage3_right_pid = Stage3LegPID()
        self.stage3_left_pid = Stage3LegPID()
        self.right_pid = LegLengthPID()
        self.left_pid = LegLengthPID()

        self.imu_cache = {}
        self.joint_cache = {}
        self.last_processed_stamp = None
        self.step_count = 0
        self.target_speed = 0.0

        # Stage 5: velocity ramp
        self.speed_setpoint = 0.0
        self.max_speed = 0.3
        self.max_accel = 0.15
        self.position_setpoint = None
        self.stage5_active = False
        self.cmd_timeout = 1.0
        self.last_cmd_time = None
        self.wheel_pi = WheelSpeedPI(
            kp=self.wheel_kp,
            ki=self.wheel_ki,
            max_torque=self.wheel_max_torque,
            dt=DT
        )


        self.publisher = self.create_publisher(
            WheelLegCmd, '/actuator_cmd', 10
        )

        self.create_subscription(
            Imu, '/imu', self.on_imu, 10
        )
        self.create_subscription(
            JointState, '/joint_states', self.on_joints, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel', self.on_velocity, 10
        )

        self.get_logger().info(
            f'VMC+LQR Controller ready; '
            f'enable_control={self.enable_control}'
        )

    def on_velocity(self, msg):
        import time

        speed = float(msg.linear.x)
        if not math.isfinite(speed):
            self.get_logger().warn("Invalid cmd_vel ignored")
            return

        self.target_speed = max(
            -self.max_speed,
            min(self.max_speed, speed)
        )
        self.last_cmd_time = time.monotonic()
        self.stage5_active = True

    def on_imu(self, msg):
        self.imu_cache[stamp_key(msg)] = msg
        self.process_pair()

    def on_joints(self, msg):
        self.joint_cache[stamp_key(msg)] = msg
        self.process_pair()

    def process_pair(self):
        common = set(self.imu_cache) & set(self.joint_cache)

        if not common:
            self.trim_cache()
            return

        stamp = max(common)

        if (
            self.last_processed_stamp is not None
            and stamp <= self.last_processed_stamp
        ):
            self.imu_cache.pop(stamp, None)
            self.joint_cache.pop(stamp, None)
            return

        imu = self.imu_cache.pop(stamp)
        joints = self.joint_cache.pop(stamp)

        self.imu_cache.clear()
        self.joint_cache.clear()
        self.last_processed_stamp = stamp

        try:
            command = self.calculate(imu, joints)
        except Exception as exc:
            self.get_logger().error(
                f'Control calculation failed: {exc}'
            )
            command = WheelLegCmd()

        self.publisher.publish(command)
        self.step_count += 1

        if self.step_count % 500 == 0:
            _, raw_pitch, _ = quaternion_to_euler(
                imu.orientation
            )
            self.get_logger().info(
                f'step={self.step_count}, '
                f'pitch={math.degrees(raw_pitch):.2f} deg, '
                f'control={self.enable_control}'
            )

    def trim_cache(self):
        for cache in (self.imu_cache, self.joint_cache):
            while len(cache) > 5:
                cache.pop(min(cache))

    def calculate(self, imu, joints):
        cmd = WheelLegCmd()


        # Stage 3: independent leg length PID + VMC
        if self.leg_length_enable:
            if len(joints.name) != len(joints.position):
                raise ValueError('JointState name/position mismatch')

            pos = dict(zip(joints.name, joints.position))
            required = ('right1', 'right2', 'left1', 'left2')
            if not all(k in pos for k in required):
                raise ValueError('Missing leg joint positions')

            if not all(math.isfinite(pos[k]) for k in required):
                raise ValueError('Non-finite leg joint position')

            _, raw_pitch, _ = quaternion_to_euler(
                imu.orientation
            )
            pitch = self.pitch_sign * raw_pitch
            gyro_pitch = (
                self.gyro_pitch_sign * imu.angular_velocity.y
            )

            if not all(math.isfinite(x) for x in
                       (pitch, gyro_pitch)):
                raise ValueError('Non-finite IMU value')

            results = []

            for flag, links, pid, m1, m2 in (
                (0, self.right_links, self.stage3_right_pid,
                 pos['right1'], pos['right2']),
                (1, self.left_links, self.stage3_left_pid,
                 pos['left1'], pos['left2'])
            ):
                links.forward_kinematics_cal(
                    flag, m1, m2, pitch, gyro_pitch
                )

                if not math.isfinite(links.L_0):
                    raise ValueError('Non-finite leg length')

                cos_theta = math.cos(links.theta)

                f_pid = pid.compute(
                    links.L_0, self.target_length
                )

                # Stage 3 临时诊断策略：
                # 接近 cos(theta)=0 时不进行重力补偿。
                if abs(cos_theta) < 0.20:
                    gravity_comp = 0.0
                else:
                    gravity_comp = 4.0 / cos_theta

                f0 = clamp(
                    gravity_comp + f_pid, 100.0
                )

                t1, t2 = links.VMC_torque_cal(f0, 0.0)

                if flag == 1:
                    t1, t2 = -t1, -t2

                if not all(math.isfinite(float(x))
                           for x in (f0, t1, t2)):
                    raise ValueError('Non-finite Stage 3 output')

                results.append(
                    (links.L_0, links.theta, f0,
                     float(t1), float(t2))
                )

            right, left = results
            cmd.right_phi1 = right[3]
            cmd.right_phi4 = right[4]
            cmd.left_phi1 = left[3]
            cmd.left_phi4 = left[4]
            cmd.right_wheel = 0.0
            cmd.left_wheel = 0.0

            if self.step_count % 500 == 0:
                self.get_logger().info(
                    f'Stage3: target={self.target_length:.4f} m, '
                    f'right_L0={right[0]:.4f} m, '
                    f'left_L0={left[0]:.4f} m, '
                    f'right_error_mm='
                    f'{(self.target_length-right[0])*1000:.2f}, '
                    f'left_error_mm='
                    f'{(self.target_length-left[0])*1000:.2f}, '
                    f'right_theta={right[1]:.3f} rad, '
                    f'left_theta={left[1]:.3f} rad, '
                    f'right_F0={right[2]:.2f} N, '
                    f'left_F0={left[2]:.2f} N'
                )

            return cmd

        # Stage 2: Wheel Speed PI
        # 此模式只测试轮速，不执行 VMC/LQR 平衡控制。
        if self.wheel_pi_enable:

            if (
                len(joints.name) != len(joints.velocity)
            ):
                raise ValueError(
                    'JointState name/velocity length mismatch'
                )

            vel = dict(zip(joints.name, joints.velocity))

            if (
                'rightwheel' not in vel
                or 'leftwheel' not in vel
            ):
                raise ValueError('Wheel velocity missing')

            if not all(
                math.isfinite(vel[name])
                for name in ('rightwheel', 'leftwheel')
            ):
                raise ValueError('Non-finite wheel velocity')

            # 根据当前项目的轮速方向计算车体线速度
            wheel_angular_speed = (
                vel['leftwheel'] - vel['rightwheel']
            ) / 2.0

            measured_speed = (
                wheel_angular_speed * WHEEL_RADIUS
            )

            wheel_torque = self.wheel_pi.compute(
                self.target_speed,
                measured_speed
            )

            # 保持阶段1验证过的左右轮力矩方向
            cmd.right_wheel = -wheel_torque
            cmd.left_wheel = wheel_torque

            if self.step_count % 500 == 0:
                error = self.target_speed - measured_speed

                self.get_logger().info(
                    f'Wheel PI: '
                    f'target={self.target_speed:.3f} m/s, '
                    f'actual={measured_speed:.3f} m/s, '
                    f'error={error:.3f} m/s, '
                    f'torque={wheel_torque:.3f} Nm'
                )

            return cmd

        # 原有 VMC + LQR 逻辑保持不变
        if not self.enable_control:
            return cmd

        if (
            len(joints.name) != len(joints.position)
            or len(joints.name) != len(joints.velocity)
        ):
            raise ValueError('JointState arrays have different lengths')

        pos = dict(zip(joints.name, joints.position))
        vel = dict(zip(joints.name, joints.velocity))

        if not all(name in pos for name in JOINT_NAMES):
            raise ValueError('Missing required joint names')

        values = (
            list(pos.values())
            + list(vel.values())
            + [
                imu.orientation.x, imu.orientation.y,
                imu.orientation.z, imu.orientation.w,
                imu.angular_velocity.y
            ]
        )

        if not all(math.isfinite(v) for v in values):
            raise ValueError('Non-finite sensor value')

        _, raw_pitch, _ = quaternion_to_euler(
            imu.orientation
        )

        pitch = self.pitch_sign * raw_pitch
        gyro_pitch = (
            self.gyro_pitch_sign * imu.angular_velocity.y
        )

        common_pos = (
            pos['leftwheel'] - pos['rightwheel']
        ) / 2.0

        common_speed = (
            vel['leftwheel'] - vel['rightwheel']
        ) / 2.0

        # Stage 5: update speed and position references once
        # per valid control cycle.
        if self.stage5_active:
            import time

            measured_x = common_pos * WHEEL_RADIUS

            if self.position_setpoint is None:
                self.position_setpoint = measured_x

            if (
                self.last_cmd_time is None
                or time.monotonic() - self.last_cmd_time
                > self.cmd_timeout
            ):
                requested_speed = 0.0
            else:
                requested_speed = self.target_speed

            max_delta = self.max_accel * DT
            delta = requested_speed - self.speed_setpoint
            delta = max(-max_delta, min(max_delta, delta))

            self.speed_setpoint += delta

            # Reference trajectory, not measured odometry.
            self.position_setpoint += self.speed_setpoint * DT

            # Both LQR controllers use the same physical
            # forward displacement convention.
            for controller in (self.right_lqr, self.left_lqr):
                controller.v_set = self.speed_setpoint
                controller.x_b_set = self.position_setpoint

        right_t1, right_t2, right_wheel = self.control_leg(
            leg_flag=0,
            links=self.right_links,
            lqr=self.right_lqr,
            pid=self.right_pid,
            motor1=pos['right1'],
            motor2=pos['right2'],
            pitch=pitch,
            gyro_pitch=gyro_pitch,
            common_pos=common_pos,
            common_speed=common_speed
        )

        left_t1, left_t2, left_wheel = self.control_leg(
            leg_flag=1,
            links=self.left_links,
            lqr=self.left_lqr,
            pid=self.left_pid,
            motor1=pos['left1'],
            motor2=pos['left2'],
            pitch=pitch,
            gyro_pitch=gyro_pitch,
            common_pos=common_pos,
            common_speed=common_speed
        )

        if self.step_count % 500 == 0:
            right_l0 = self.right_links.L_0
            left_l0 = self.left_links.L_0
            right_error = (
                self.target_length - right_l0
            ) * 1000.0
            left_error = (
                self.target_length - left_l0
            ) * 1000.0

            self.get_logger().info(
                f'BalanceLeg: '
                f'target={self.target_length:.4f} m, '
                f'right_L0={right_l0:.5f} m, '
                f'left_L0={left_l0:.5f} m, '
                f'right_error_mm={right_error:.2f}, '
                f'left_error_mm={left_error:.2f}, '
                f'right_theta={self.right_links.theta:.5f} rad, '
                f'left_theta={self.left_links.theta:.5f} rad, '
                f'right_ok={abs(right_error) <= 5.0}, '
                f'left_ok={abs(left_error) <= 5.0}'
            )

        outputs = (
            right_t1, right_t2,
            left_t1, left_t2,
            right_wheel, left_wheel
        )

        if not all(math.isfinite(v) for v in outputs):
            raise ValueError('Non-finite control output')

        cmd.right_phi1 = clamp(right_t1, 6.0)
        cmd.right_phi4 = clamp(right_t2, 6.0)
        cmd.left_phi1 = clamp(left_t1, 6.0)
        cmd.left_phi4 = clamp(left_t2, 6.0)
        cmd.right_wheel = clamp(right_wheel, 2.0)
        cmd.left_wheel = clamp(left_wheel, 2.0)

        return cmd

    def control_leg(
        self, leg_flag, links, lqr, pid,
        motor1, motor2, pitch, gyro_pitch,
        common_pos, common_speed
    ):
        links.forward_kinematics_cal(
            leg_flag, motor1, motor2,
            pitch, gyro_pitch
        )

        # 原程序右侧传入反向共模值。
        if leg_flag == 0:
            motor_pos = -common_pos
            motor_speed = -common_speed
        else:
            motor_pos = common_pos
            motor_speed = common_speed

        lqr.states_update(
            leg_flag, links, pitch, gyro_pitch,
            motor_pos, motor_speed, WHEEL_RADIUS
        )

        K = lqr.calc_k_matrix_from_poly(links.L_0)

        F_ctrl = pid.compute(
            links.L_0, self.target_length
        )

        cos_theta = math.cos(links.theta)
        if abs(cos_theta) < 0.05:
            raise ValueError('Leg angle too close to singularity')

        F_0 = clamp(4.0 / cos_theta + F_ctrl, 100.0)

        state = np.array([
            lqr.theta,
            lqr.d_theta,
            lqr.x_b - lqr.x_b_set,
            lqr.v_b_whole - lqr.v_set,
            lqr.phi,
            lqr.d_phi
        ])

        wheel_T = float(np.dot(K[0::2], state))
        T_p = float(np.dot(K[1::2], state))

        t1, t2 = links.VMC_torque_cal(F_0, T_p)

        if leg_flag == 0:
            return float(t1), float(t2), clamp(wheel_T, 1.5)

        return -float(t1), -float(t2), clamp(-wheel_T, 1.5)


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
