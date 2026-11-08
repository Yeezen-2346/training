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
        self.kd = 5.0
        self.last_error = 0.0

    def compute(self, current, target):
        error = target - current
        derivative = error - self.last_error
        self.last_error = error
        return clamp(self.kp * error + self.kd * derivative, 150.0)


class WheelLegController(Node):
    def __init__(self):
        super().__init__('wheel_leg_controller')

        self.declare_parameter('enable_control', False)
        self.declare_parameter('target_length', 0.10)
        self.declare_parameter('pitch_sign', -1.0)
        self.declare_parameter('gyro_pitch_sign', -1.0)

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

        self.right_links = five_links_param(DT)
        self.left_links = five_links_param(DT)

        self.right_lqr = lqr_controller(DT)
        self.left_lqr = lqr_controller(DT)

        self.right_pid = LegLengthPID()
        self.left_pid = LegLengthPID()

        self.imu_cache = {}
        self.joint_cache = {}
        self.last_processed_stamp = None
        self.step_count = 0
        self.target_speed = 0.0

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
        # 首轮站立测试暂不引入运动指令。
        self.target_speed = float(msg.linear.x)

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
