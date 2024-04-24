#!/usr/bin/env python
# coding:utf-8
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from custom_msgs.msg import Motors_vel
from custom_msgs.msg import Thruster_pwm
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

class MPCController():
    def __init__(self, wheel_diameter=None, wheel_separation=None):
        
        self._is_init = False
        # 获取车轮直径和修正间距
        if rospy.has_param('~wheel_diameter'):
            self.wheel_diameter = rospy.get_param('~wheel_diameter')
        if rospy.has_param('~wheel_separation'):
            self.wheel_separation = rospy.get_param('~wheel_separation')
        self.wheel_diameter = wheel_diameter
        self.wheel_separation = wheel_separation

        if not self.wheel_diameter or not self.wheel_separation:
            raise rospy.ROSException('wheel_diameter and wheel_separation are None')
        
        # 小车电机最大转速
        self.maxRPM = 500

        # 运动学模型状态矩阵
        self.A = np.array([[0.5, 0.5], [1/self.wheel_separation, -1/self.wheel_separation]])

        # 实体车轮速度分配矩阵
        if rospy.has_param('~wheel_vel_distribution'):
            distribution = rospy.get_param('~wheel_vel_distribution')
            self.wheel_vel_distribution = np.array(distribution).reshape(-1,2)
        else:
            self.wheel_vel_distribution = np.array([[1, 0], [1, 0], [0, 1], [0, 1]])

        self.inv_wheel_vel_distribution = np.linalg.pinv(self.wheel_vel_distribution)
            

        # 发布电机命令
        self._guide_motors_vel_pub = rospy.Publisher('guide_Motors_vel', Motors_vel, queue_size=1)
        # 订阅姿态和电机转速
        self._attitude_sub = rospy.Subscriber('attitude_inEuler', Vector3Stamped, self._imu_callback)
        self._motors_vel_sub = rospy.Subscriber('motors_vel_inrpm', Motors_vel, self._motors_vel_callback)
        self._ref_odom_sub = rospy.Subscriber('ref_odometry', Odometry, self._odom_callback)

        # 小车状态
        self.factor_rpmtomps = np.pi/30*self.wheel_diameter # rpm --> m/s in wheels
        self.left_wheel_vel = 0
        self.right_wheel_vel = 0
        self.left_right_wheels_vel = np.zeros((2, 1)) # unit:m/s
        self.current_attitude = Vector3Stamped() # unit:degree
        self.motors_vel = Motors_vel() # unit:rpm

        # 参考里程计信息
        self.ref_odometry = Odometry()
        self.current_ref_vel = np.zeros((2, 1))
        self.current_ref_psi = np.zeros((1, 1))

        # mpc超参数
        self.N = 20
        self.dt = 0.05
        self.Q_vel = np.diag([1, 0]) * 1e5
        self.Q_psi = np.diag([1]) * 1e5
        self.R = np.diag([1, 1]) * 0
        # mpc输出限幅
        self.saturation = self.maxRPM * self.factor_rpmtomps
        # 初始化mpc求解器
        self.opti = ca.Opti()
        self.U = self.opti.variable(2, self.N) # 左右轮线速度
        self.X_vel = self.opti.variable(2, self.N) # 线速度和角速度
        self.X_psi = self.opti.variable(1, self.N+1)
        self.ref_vel = self.opti.parameter(2, 1)
        self.ref_psi = self.opti.parameter(1, 1)
        self.X0_psi = self.opti.parameter(1, 1)
        J = 0
        for i in range(self.N):
            self.opti.subject_to(self.X_vel[:, i] == self.A @ self.U[:, i])
            self.opti.subject_to(self.X_psi[:, i+1] == self.X_psi[:, i] + self.X_vel[1, i] * self.dt)

            J += (self.X_vel[:, i] - self.ref_vel).T @ self.Q_vel @ (self.X_vel[:, i] - self.ref_vel) + \
                 (self.X_psi[:, i+1] - self.ref_psi).T @ self.Q_psi @ (self.X_psi[:, i+1] - self.ref_psi) + \
                  self.U[:, i].T @ self.R @ self.U[:, i]

        self.opti.subject_to(self.X_psi[:, 0] == self.X0_psi)
        self.opti.subject_to(self.opti.bounded(-self.saturation, ca.vec(self.U), self.saturation))

        self.opti.minimize(J)
        opts={}
        opts["print_time"] = False
        s_opts = {}
        s_opts["print_level"] = 0
        self.opti.solver("ipopt", opts, s_opts)

        self._is_init = True

    # 四元数转欧拉角, 返回弧度
    def quaternion2euler(quaternion):
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz')
        return euler

    def _imu_callback(self, msg):
        if not isinstance(msg, Vector3Stamped):
            raise rospy.ROSException('imu data instance is not Vector3Stamped')
        self.current_attitude = msg

        self.update_controller()

    def _motors_vel_callback(self, msg):
        if not isinstance(msg, Motors_vel):
            raise rospy.ROSException('motors data instance is not Motors_vel')
        self.motors_vel = msg

    def _odom_callback(self, msg):
        self.ref_odometry = msg
        
                
    def publish_motors_output(self, output):
        motors_vel_vectors = self.wheel_vel_distribution @ output / self.factor_rpmtomps
        motors_vel = Motors_vel()
        motors_vel.motor1_rpm = motors_vel_vectors[0]
        motors_vel.motor2_rpm = motors_vel_vectors[1]
        motors_vel.motor3_rpm = motors_vel_vectors[2]
        motors_vel.motor4_rpm = motors_vel_vectors[3]

        self._guide_motors_vel_pub.publish(motors_vel)


        
    def update_controller(self):
        if not self._is_init:
            return
        
        self.current_ref_vel[0] = self.ref_odometry.twist.twist.linear.x
        self.current_ref_vel[1] = self.ref_odometry.twist.twist.angular.z
        self.current_ref_psi = self.quaternion2euler(self.ref_odometry.pose.pose.orientation)[2]
        # mpc控制器更新
        self.opti.set_value(self.X0_psi, self.current_attitude.vector.z * 180 / np.pi)
        self.opti.set_value(self.ref_psi, self.current_ref_psi)
        self.opti.set_value(self.ref_vel, self.current_ref_vel)
        sol = self.opti.solve()
        wheels_output = sol.value(self.U[:, 0])

        self.publish_motors_output(wheels_output)


if __name__ == '__main__':
    print('MPCControlle start')
    rospy.init_node('mpc_controller_node')

    try:
        node = MPCController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')








