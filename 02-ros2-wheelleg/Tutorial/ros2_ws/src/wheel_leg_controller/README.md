#介绍
## 阶段一 1 打通链路
### 任务
控制器先只发固定值/正弦，把三条话题跑起来
### 验收
#### 1./joint_states有稳定频率
##### 执行命令：
ros2 topic echo /joint_states --once
##### 收到的连续反馈：
  stamp:
    sec: 1794555277
    nanosec: 837470540
  frame_id: ''
name:
- right1
- right2
- left1
- left2
- rightwheel
- leftwheel
position:
- -0.36932723244805205
- 0.4327899299539949
- 0.36932720178386486
- -0.43279001348200535
- -1.2426214509980524
- 1.2350543594635734
velocity:
- -5.422550095823559e-11
- 1.169015766121163e-10
- 7.866432748164965e-11
- -1.7330396128995683e-10
- 2.513734714765995e-11
- -2.808663460715159e-11
effort: []

#### 2.actuator_cmd有稳定频率
##### 执行命令：
ros2 topic pub -r 50 /actuator_cmd rm_msgs/msg/WheelLegCmd \
'{right_phi1: 0.0, right_phi4: 0.0, left_phi1: 0.0, left_phi4: 0.0, right_wheel: -0.05, left_wheel: 0.05}'
##### 收到的连续反馈：
publishing #1554: rm_msgs.msg.WheelLegCmd(right_phi1=0.0, right_phi4=0.0, left_phi1=0.0, left_phi4=0.0, right_wheel=-0.05, left_wheel=0.05)

publishing #1555: rm_msgs.msg.WheelLegCmd(right_phi1=0.0, right_phi4=0.0, left_phi1=0.0, left_phi4=0.0, right_wheel=-0.05, left_wheel=0.05)
#### 3.轮子有响应
##### 执行命令：
ros2 topic echo /joint_states
##### 收到的数据：
name:
- right1
- right2
- left1
- left2
- rightwheel
- leftwheel

velocity:
- -1.377769e-05
- -2.357151e-05
-  1.377765e-05
-  2.357144e-05
- -0.014509
-  0.014509

## 阶段二 2 轮速控制
### 执行命令：
#### 1.配置参数
ros2 run wheel_leg_controller wheel_leg_controller \
--ros-args \
-p enable_control:=false \
-p wheel_pi_enable:=true \
-p wheel_kp:=1.2 \
-p wheel_ki:=2.0 \
-p wheel_max_torque:=1.2
#### 2.发布目标速度
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}}"
###  收到的数据：
[INFO] [1794558520.705427689] [wheel_leg_controller]: Wheel PI: target=0.500 m/s, actual=0.520 m/s, error=-0.020 m/s, torque=1.083 Nm
[INFO] [1794558520.971812870] [wheel_leg_controller]: step=159000, pitch=52.04 deg, control=False
[INFO] [1794558520.972668568] [wheel_leg_controller]: Wheel PI: target=0.500 m/s, actual=0.481 m/s, error=0.019 m/s, torque=1.127 Nm
[INFO] [1794558521.234663640] [wheel_leg_controller]: step=159500, pitch=52.00 deg, control=False
[INFO] [1794558521.235478848] [wheel_leg_controller]: Wheel PI: target=0.500 m/s, actual=0.465 m/s, error=0.035 m/s, torque=1.143 Nm
[INFO] [1794558521.553449211] [wheel_leg_controller]: step=160000, pitch=51.96 deg, control=False
[INFO] [1794558521.554084789] [wheel_leg_controller]: Wheel PI: target=0.500 m/s, actual=0.574 m/s, error=-0.074 m/s, torque=1.016 Nm
[INFO] [1794558521.840398916] [wheel_leg_controller]: step=160500, pitch=51.95 deg, control=False
[INFO] [1794558521.841042123] [wheel_leg_controller]: Wheel PI: target=0.500 m/s, actual=0.508 m/s, error=-0.008 m/s, torque=1.093 Nm
[INFO] [1794558522.189794428] [wheel_leg_controller]: step=161000, pitch=52.01 deg, control=False
[INFO] [1794558522.190281711] [wheel_leg_controller]: Wheel PI: target=0.500 m/s, actual=0.444 m/s, error=0.056 m/s, torque=1.169 Nm
[INFO] [1794558522.512379498] [wheel_leg_controller]: step=161500, pitch=52.04 deg, control=False
[INFO] [1794558522.512900861] [wheel_leg_controller]: Wheel PI: target=0.500 m/s, actual=0.453 m/s, error=0.047 m/s, torque=1.162 Nm
### 结论
经过计算，误差均小于 20%，围绕目标速度 0.5 m/s 波动

## 阶段三  VMC 接入
### 1 用腿关节角计算 L_0
#### 执行的命令
ros2 topic echo /joint_states --once
#### 结果
name:
- right1
- right2
- left1
- left2
- rightwheel
- leftwheel

position:
- 4.0252713836433625
- 0.6435877723763437
- -4.025269614929236
- -0.6435690026952829
- -1.5058504485340738
- 1.5111727874017056
#### 结论
证明仿真节点向控制器提供了左右腿关节角度

### 2 计算theta
#### 收到的数据
[INFO] [1794730534.872278214] [wheel_leg_controller]: BalanceLeg: target=0.1000 m, right_L0=0.09863 m, left_L0=0.09863 m, right_error_mm=1.37, left_error_mm=1.37, right_theta=0.02278 rad, left_theta=0.02278 rad, right_ok=True, left_ok=True
[INFO] [1794730535.160604643] [wheel_leg_controller]: step=11000, pitch=0.11 deg, control=True
[INFO] [1794730535.161286366] [wheel_leg_controller]: BalanceLeg: target=0.1000 m, right_L0=0.09866 m, left_L0=0.09866 m, right_error_mm=1.34, left_error_mm=1.34, right_theta=0.02270 rad, left_theta=0.02270 rad, right_ok=True, left_ok=True
[INFO] [1794730535.523505778] [wheel_leg_controller]: step=11500, pitch=0.11 deg, control=True
[INFO] [1794730535.524127028] [wheel_leg_controller]: BalanceLeg: target=0.1000 m, right_L0=0.09869 m, left_L0=0.09869 m, right_error_mm=1.31, left_error_mm=1.31, right_theta=0.02284 rad, left_theta=0.02284 rad, right_ok=True, left_ok=True
[INFO] [1794730535.964222717] [wheel_leg_controller]: step=12000, pitch=0.11 deg, control=True
[INFO] [1794730535.964780347] [wheel_leg_controller]: BalanceLeg: target=0.1000 m, right_L0=0.09872 m, left_L0=0.09872 m, right_error_mm=1.28, left_error_mm=1.28, right_theta=0.02308 rad, left_theta=0.02308 rad, right_ok=True, left_ok=True
#### 结论
能计算theta

### 3 腿力矩
#### 执行的命令
ros2 topic echo /actuator_cmd --once
#### 收到的数据
right_phi1: 0.5234886853956152
right_phi4: -0.9192710725924522
left_phi1: -0.5233892713674493
left_phi4: 0.9191519851984403
right_wheel: 0.0
left_wheel: 0.0
#### 结论
VMC能输出腿关节力矩

### 4 腿长 PID
#### 执行的命令
ros2 run wheel_leg_controller wheel_leg_controller \
  --ros-args \
  -p enable_control:=true \
  -p wheel_pi_enable:=false \
  -p leg_length_enable:=false \
  -p target_length:=0.10 \
  -p pitch_sign:=-1.0 \
  -p gyro_pitch_sign:=-1.0
####收到的数据
step=26000, pitch=0.11 deg, control=True

BalanceLeg:
target=0.1000 m,
right_L0=0.09933 m,
left_L0=0.09933 m,
right_error_mm=0.67,
left_error_mm=0.67,
right_ok=True,
left_ok=True
#### 结论
可输出腿长PID

### 总结论
L_0 稳定在目标值 ±5 mm
17 秒观测中，左右腿长度均保持在允许范围内，误差为 0.67 mm。机身俯仰角维持在约 0.10°～0.11°，机器人保持稳定

## 阶段4 LQR 平衡
### 1 控制结构
从IMU（pitch、gyro_pitch）和轮里程计（轮位置、轮速度）
到lqr.states_update()更新六维状态
再到LQR 状态反馈（K(L₀) × 状态向量）
最后输出左右轮力矩 + VMC 腿力矩

### 2 K/参数来源
#### LQR 增益 K 的来源
定义了 12 组三次多项式系数，每个反馈增益根据当前虚拟腿长计算：
\[
K_i(L_0)=a_iL_0^3+b_iL_0^2+c_iL_0+d_i,\quad i=0,\ldots,11
\]
程序通过 `calc_k_matrix_from_poly(L_0)` 实时计算增益向量，并将其中偶数索引的六个增益用于轮力矩控制，奇数索引的六个增益用于虚拟腿部力矩控制。
#### LQR 状态反馈参数
LQR 使用六维状态向量：
\[
x=[\theta,\dot{\theta},x_b-x_{b,set},v_b-v_{set},\phi,\dot{\phi}]^T
\]
- \(\theta,\dot{\theta}\)：通过五连杆运动学计算的虚拟腿角度及角速度。
- \(x_b,v_b\)：根据轮关节位置、速度及轮半径估计的位移和速度。
- \(\phi,\dot{\phi}\)：通过 IMU 姿态和陀螺仪获取的机身俯仰角及角速度。
轮力矩与虚拟腿部力矩分别由相应的 LQR 反馈增益和状态向量相乘得到。
#### 其他控制参数来源
轮半径设为 0.06 m，与控制器中的 `WHEEL_RADIUS` 一致。
目标腿长设为 0.10 m，对应控制器参数 `target_length`。
为统一 IMU 测量方向与控制器坐标系，`pitch_sign` 和 `gyro_pitch_sign` 均设为 -1.0。
腿长 PID 采用 Kp=700、Ki=10、Kd=5。其中 Kp 和 Kd 延续已有控制器设置，Ki 在本阶段调试中设为 10，改善腿长稳态误差。
#### 参数来源说明
留了参考工程中已有的 MATLAB LQR 拟合系数，主要工作是将其接入 ROS2 控制器，完成 IMU、轮里程计和五连杆状态的更新，并结合 VMC 与腿长 PID 实现平衡控制。

### 3 Pitch 曲线和ROS bag
- pitch_curve.png：pitch 随记录时间变化的曲线
- stage4_lqr_balance/：ROS2 原始 bag 数据

## 阶段五
### 录制
/home/venom-chenyizhen/venom_cpp_basics/training/02-ros2-wheelleg/Tutorial/ros2_ws/src/wheel_leg_controller/evidence/stage5_velocity/stage5_0p3ms_drive.mp4
