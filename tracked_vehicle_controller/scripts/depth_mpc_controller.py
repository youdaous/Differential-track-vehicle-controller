#!/usr/bin/env python
# coding:utf-8
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from custom_msgs.msg import HeightDepth
from custom_msgs.msg import Thruster_pwm
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DepthMPCController():
    def __init__(self):
        
        self._is_init = False
        # 获取动力学模型参数
        if rospy.has_param('~MassMatrix'):
            MassMatrix = rospy.get_param('~MassMatric')
            self.MassMatrix = np.diag(MassMatrix)
            print(self.MassMatric)
        if rospy.has_param('~DampingMatrix'):
            DampingMatrix = rospy.get_param('~DampingMatrix')
            self.DampingMatrix = np.diag(DampingMatrix)
        if rospy.has_param('~g'):
            self.g = rospy.get_param('~g')

        self.invMassMatrix = np.linalg.inv(MassMatrix)

        
        # 控制分配阵
        self.MixControl = np.array([[1, 1], [1, -1]])

        # 发布推进器命令
        self._guide_trusters_pwm_pub = rospy.Publisher('guide_Thrusters_pwm', Thruster_pwm, queue_size=1)
        # 订阅姿态
        self._attitude_sub = rospy.Subscriber('attitude_inEuler', Vector3Stamped, self._imu_callback)
        # 订阅高度深度
        self._heightDepth_sub = rospy.Subscriber('Height_Depth', HeightDepth, self._heightDepth_callback)
        self._ref_odom_sub = rospy.Subscriber('ref_odometry', Odometry, self._ref_odom_callback)

        # 小车状态
        self.current_attitude = Vector3Stamped() # unit:degree
        self.current_heightDepth = HeightDepth()
        self.current_vehicle_pose = np.zeros((2, 1))
        self.current_vehicle_vel = np.zeros((2, 1))

        # 参考里程计信息
        self.ref_odometry = Odometry()
        self.current_ref_pose = np.zeros((2, 1))
        self.current_ref_V = np.zeros((2, 1))

        # mpc参数
        self.N = 20
        self.dt = 0.5
        self.Q = np.diag([1, 1]) * 1e5
        self.R = np.diag([1, 1])

        # mpc输出限幅
        self.saturation = 500 # unit:N
        # 初始化mpc求解器
        self.opti = ca.Opti()
        self.U = self.opti.variable(2, self.N) # [force_z, torque_x]
        self.X_vel = self.opti.variable(2, self.N+1) # [u, omega]
        self.X_pose = self.opti.variable(2, self.N+1)  # [z, phi]
        self.ref_pose = self.opti.parameter(2, 1)
        self.X0_pose = self.opti.parameter(2, 1)
        self.X0_vel = self.opti.parameter(2, 1)

        J = 0
        for i in range(self.N):
            phi = self.X_pose[1, 0]
            
            veltans_MX = ca.MX(2,2)
            veltans_MX[0,0] = ca.cos(phi)
            veltans_MX[0,1] = 0
            veltans_MX[1,0] = 0
            veltans_MX[1,1] = 1
            self.opti.subject_to(self.X_vel[:, i+1] == self.X_vel[:, i] + self.dt * self.invMassMatrix @ \
                                 (self.U[:, i] - (self.DampingMatrix @ self.X_vel[:, i]) - self.g))
            self.opti.subject_to(self.X_pose[:, i+1] == self.X_pose[:, i] + veltans_MX @ self.X_vel[:, i] * self.dt)

            error_pose = self.X_pose[:, i+1] - self.ref_pose
            J += error_pose.T @ self.Q @ error_pose + \
                 self.U[:, i].T @ self.R @ self.U[:, i]

        self.opti.subject_to(self.X_pose[:, 0] == self.X0_pose)
        self.opti.subject_to(self.X_vel[:, 0] == self.X0_vel)
        self.opti.subject_to(self.opti.bounded(-self.saturation, ca.vec(self.U), self.saturation))

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
    
    # 车辆高度深度
    def _heightDepth_callback(self, msg):
        self.current_heightDepth = msg

    def _imu_callback(self, msg):
        if not isinstance(msg, Vector3Stamped):
            raise rospy.ROSException('imu data instance is not Vector3Stamped')
        self.current_attitude = msg
        self.update_controller()

    def _ref_odom_callback(self, msg):
        self.ref_odometry = msg  

    def publish_thruster_pwm(self, thruster_force):
        thruster_value = self.MixControl @ thruster_force / (2 * self.saturation) # 归一化和控制分配
        thruster_pwm = thruster_value * 400 + 1500 # 映射为pwm:1100～1900
        thruster_pubmsg = Thruster_pwm()
        thruster_pubmsg.pwm1 = thruster_pwm[0]
        thruster_pubmsg.pwm2 = thruster_pwm[1]
        
        self._guide_trusters_pwm_pub.publish(thruster_pubmsg)

    # 控制器更新
    def update_controller(self):
        if not self._is_init:
            return
        
        self.current_vehicle_pose = np.array([self.current_heightDepth.strong_target_dist, self.current_attitude.vector.x]).reshape(-1, 1)
        
        # 参考航向与车辆当前航向奇异的解决
        if abs(self.current_ref_pose[1] - self.current_vehicle_pose[1]) > np.pi:
            if self.current_ref_pose[1] > 0:
                self.current_ref_pose[1] = -2 * np.pi + self.current_ref_pose[1]
            else:
                self.current_ref_pose[1] = 2 * np.pi + self.current_ref_pose[1]

        # mpc控制器更新
        self.opti.set_value(self.X0_pose, self.current_vehicle_pose)
        self.opti.set_value(self.X0_vel, self.current_vehicle_vel)
        self.opti.set_value(self.ref_pose, self.current_ref_pose)
        sol = self.opti.solve()
        thrusters_output = sol.value(self.U[:, 0])

        self.publish_thruster_pwm(thruster_force=thrusters_output)




if __name__ == '__main__':
    print('MPCControlle start')
    rospy.init_node('mpc_controller_node')

    try:
        node = DepthMPCController()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')








