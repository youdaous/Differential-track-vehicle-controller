#!/usr/bin/env python
# coding:utf-8
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from custom_msgs.msg import Motors_vel
from custom_msgs.msg import Thruster_pwm
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MMPCController():
    def __init__(self, wheel_diameter=None, wheel_separation=None):
        
        self._is_init = False
        self.wheel_diameter = wheel_diameter
        self.wheel_separation = wheel_separation
        # 获取车轮直径和修正间距
        if rospy.has_param('~wheel_diameter'):
            self.wheel_diameter = rospy.get_param('~wheel_diameter')
            print(self.wheel_diameter)
        if rospy.has_param('~wheel_separation'):
            self.wheel_separation = rospy.get_param('~wheel_separation')

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
        self._ref_odom_sub = rospy.Subscriber('ref_odometry', Odometry, self._ref_odom_callback)

        # 仿真中订阅小车里程计
        self._odom_sub = rospy.Subscriber('odom', Odometry, self._odom_callback)
        self.odomInsim = Odometry()
        # 仿真中发布速度
        self._vel_cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # 小车状态
        self.factor_rpmtomps = np.pi/30*self.wheel_diameter # rpm --> m/s in wheels
        self.left_wheel_vel = 0
        self.right_wheel_vel = 0
        self.left_right_wheels_vel = np.zeros((2, 1)) # unit:m/s
        self.current_attitude = Vector3Stamped() # unit:degree
        self.motors_vel = Motors_vel() # unit:rpm

        # 参考里程计信息
        self.ref_odometry = Odometry()
        self.current_ref_X = np.zeros((3, 1))
        self.current_ref_V = np.zeros((2, 1))
        # self.current_ref_X[2] = np.pi/2
        self.current_ref_V[0] = 0.2
        self.current_ref_X[2] = -np.pi * 0.75
        # self.current_ref_X[0] = 0
        # self.current_ref_X[1] = 5

        # mpc参数
        self.N = 20
        self.dt = 0.5
        self.Q = np.diag([0, 0, 1]) * 1e5
        self.R = np.diag([1, 0]) * 1e5
        self.vehicle_Xstate = np.zeros((3, 1))
        # mpc输出限幅
        self.saturation = self.maxRPM * self.factor_rpmtomps
        # 初始化mpc求解器
        self.opti = ca.Opti()
        self.V = self.opti.variable(2, self.N) # [u, omega]
        self.X = self.opti.variable(3, self.N+1)  # [x, y, psi]
        self.ref_X = self.opti.parameter(3, 1)
        self.ref_V = self.opti.parameter(2, 1)
        self.X0 = self.opti.parameter(3, 1)

        J = 0
        for i in range(self.N):
            theta = self.X[1, 0]
            
            veltans_MX = ca.MX(3,2)
            veltans_MX[0,0] = ca.cos(theta)
            veltans_MX[0,1] = 0
            veltans_MX[1,0] = ca.sin(theta)
            veltans_MX[1,1] = 0
            veltans_MX[2,0] = 0
            veltans_MX[2,1] = 1
            self.opti.subject_to(self.X[:, i+1] == self.X[:, i] + veltans_MX @ self.V[:, i] * self.dt)

            # veltrans = np.array([[ca.cos(theta), 0], [ca.sin(theta), 0], [0, 1]])
            # for j in range(3):
            #     self.opti.subject_to(self.X[j, i+1] == self.X[j, i] + (veltrans[j, 0] * self.V[0, i] + \
            #                                                            veltrans[j, 1] * self.V[1, i]) * self.dt)
            error_X = self.X[:, i+1] - self.ref_X
            error_V = self.V[:, i] - self.ref_V
            J += error_X.T @ self.Q @ error_X + \
                 error_V.T @ self.R @ error_V

        self.opti.subject_to(self.X[:, 0] == self.X0)
        self.opti.subject_to(self.opti.bounded(-self.saturation, ca.vec(self.V), self.saturation))

        self.opti.minimize(J)
        opts={}
        opts["print_time"] = False
        s_opts = {}
        s_opts["print_level"] = 0
        self.opti.solver("ipopt", opts, s_opts)

        self._is_init = True

    # 四元数转欧拉角, 返回弧度
    def quaternion2euler(self, quaternion):
        r = R.from_quat(np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w]))
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

    def _ref_odom_callback(self, msg):
        self.ref_odometry = msg

    def _odom_callback(self, msg):
        self.odomInsim = msg
        self.update_controller()
        
                
    def publish_motors_output(self, output):
        motors_vel_vectors = self.wheel_vel_distribution @ output / self.factor_rpmtomps
        motors_vel = Motors_vel()
        motors_vel.motor1_rpm = motors_vel_vectors[0]
        motors_vel.motor2_rpm = motors_vel_vectors[1]
        motors_vel.motor3_rpm = motors_vel_vectors[2]
        motors_vel.motor4_rpm = motors_vel_vectors[3]

        self._guide_motors_vel_pub.publish(motors_vel)

    def publish_sim_velcmd(self, output):
        vel_cmd = Twist()
        vel_cmd.linear.x = output[0]
        vel_cmd.angular.z = output[1]
        self._vel_cmd_pub.publish(vel_cmd)
        
    def update_controller(self):
        if not self._is_init:
            return
        
        # self.current_ref_vel[0] = self.ref_odometry.twist.twist.linear.x
        # self.current_ref_vel[1] = self.ref_odometry.twist.twist.angular.z
        # self.current_ref_psi = self.quaternion2euler(self.ref_odometry.pose.pose.orientation)[2]
        self.vehicle_Xstate = np.array([self.odomInsim.pose.pose.position.x, self.odomInsim.pose.pose.position.y, \
                            self.quaternion2euler(self.odomInsim.pose.pose.orientation)[2]]).reshape(3, 1)
        
        # 参考航向与车辆当前航向奇异的解决
        if abs(self.current_ref_X[2] - self.vehicle_Xstate[2]) > np.pi:
            if self.current_ref_X[2] > 0:
                self.current_ref_X[2] = -2 * np.pi + self.current_ref_X[2]
            else:
                self.current_ref_X[2] = 2 * np.pi + self.current_ref_X[2]

        # mpc控制器更新
        self.opti.set_value(self.X0, self.vehicle_Xstate)
        self.opti.set_value(self.ref_X, self.current_ref_X)
        self.opti.set_value(self.ref_V, self.current_ref_V)
        sol = self.opti.solve()
        vels_output = sol.value(self.V[:, 0])

        # self.publish_motors_output(vels_output)
        self.publish_sim_velcmd(vels_output)


if __name__ == '__main__':
    print('MPCControlle start')
    rospy.init_node('mpc_controller_node')

    try:
        node = MMPCController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')








