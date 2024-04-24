#include <casadi/casadi.hpp>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <custom_msgs/Motors_vel.h>
#include <custom_msgs/Thruster_pwm.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MPCController {
public:
    MPCController(ros::NodeHandle& nodehandle, double wheel_diameter, double wheel_separation) 
        : nh_(nodehandle), wheel_diameter_(wheel_diameter), wheel_separation_(wheel_separation) {
        if (ros::param::has("~wheel_diameter")) {
            ros::param::get("~wheel_diameter", wheel_diameter_);
        }
        if (ros::param::has("~wheel_separation")) {
            ros::param::get("~wheel_separation", wheel_separation_);
        }

        if (wheel_diameter_ == 0 || wheel_separation_ == 0) {
            throw ros::Exception("wheel_diameter and wheel_separation cannot be zero");
        }

        max_rpm_ = 500;
        A_ << 0.5, 0.5,
              1/wheel_separation_, -1/wheel_separation_;

        wheel_vel_distribution_ << 1, 0,
                                   1, 0,
                                   0, 1,
                                   0, 1;

        inv_wheel_vel_distribution_ = wheel_vel_distribution_.completeOrthogonalDecomposition().pseudoInverse();

        factor_rpmtomps_ = M_PI / 30 * wheel_diameter_;
        // Initialize ROS publishers and subscribers
        guide_motors_vel_pub_ = nh_.advertise<custom_msgs::Motors_vel>("guide_Motors_vel", 1);
        attitude_sub_ = nh_.subscribe("attitude_inEuler", 1, &MPCController::_imu_callback, this);
        motors_vel_sub_ = nh_.subscribe("motors_vel_inrpm", 1, &MPCController::_motors_vel_callback, this);
        ref_odom_sub_ = nh_.subscribe("ref_odometry", 1, &MPCController::_odom_callback, this);

        //initialization of MPC opti
        N = 20;
        dt = 0.05;
        Q_vel = casadi::DM(2, 2);
        Q_vel(0, 0) = 1e3;
        Q_vel(1, 1) = 0;
        Q_psi = casadi::DM(1, 1);
        Q_psi(0, 0) = 1e5;
        R = casadi::DM(2, 2);
        R(0, 0) = 0;
        R(1, 1) = 0;
        // MPC output saturation
        saturation = maxRPM * factor_rpmtomps_;
        
        // Initialize MPC solver
        opti = casadi::Opti();
        U = opti.variable(2, N); // Left and right wheel velocities
        X_vel = opti.variable(2, N); // Linear and angular velocities
        X_psi = opti.variable(1, N + 1);
        ref_vel = opti.parameter(1, 1);
        ref_psi = opti.parameter(1, 1);
        X0_psi = opti.parameter(1, 1);
        
        casadi::MX J = 0;
        for (int i = 0; i < N; ++i) {
            opti.subject_to(X_vel(0, i) == A_(0, 0) * U(0, i) + A_(0, 1) * U(1, i));
            opti.subject_to(X_vel(1, i) == A_(1, 0) * U(0, i) + A_(1, 1) * U(1, i));
            opti.subject_to(X_psi(0, i + 1) == X_psi(0, i) + X_vel(1, i) * dt);
            
            casadi::MX e1 = X_vel(0, i) - ref_vel(0);
            casadi::MX e2 = X_psi(0, i + 1) - ref_psi(0);
            J += (e1.T() * Q_vel(0, 0) * e1) + (e2.T() * Q_psi * e2) + (U(0, i) * R(0, 0) * U(0, i)) + (U(1, i) * R(1, 1) * U(1, i));
        }
        
        opti.subject_to(X_psi(0, 0) == X0_psi(0));
        opti.subject_to(-saturation<=U<=saturation);
        
        opti.minimize(J);
        
        casadi::Dict opts;
        opts["print_time"] = false;
        casadi::Dict s_opts;
        s_opts["print_level"] = 0;
        opti.solver("ipopt", opts, s_opts);
        
        _is_init = true;

    }

private:
    ros::NodeHandle nh_;
    ros::Publisher guide_motors_vel_pub_;
    ros::Subscriber attitude_sub_;
    ros::Subscriber motors_vel_sub_;
    ros::Subscriber ref_odom_sub_;

    double wheel_diameter_;
    double wheel_separation_;
    double max_rpm_;
    double factor_rpmtomps_;
    Eigen::MatrixXd A_;
    Eigen::MatrixXd wheel_vel_distribution_;
    Eigen::MatrixXd inv_wheel_vel_distribution_;

    casadi::Slice all;
    int N;
    double dt;
    casadi::DM Q_vel;
    casadi::DM Q_psi;
    casadi::DM R;
    double maxRPM;
    double saturation;
    casadi::Opti opti;
    casadi::MX U;
    casadi::MX X_vel;
    casadi::MX X_psi;
    casadi::MX ref_vel;
    casadi::MX ref_psi;
    casadi::MX X0_psi;
    bool _is_init = false;

    double current_ref_vel;
    double current_ref_psi;



    geometry_msgs::Vector3Stamped current_attitude_;
    nav_msgs::Odometry ref_odometry_;
    custom_msgs::Motors_vel motors_vel_;

    // Quaternion to Euler angles conversion
    Eigen::Vector3d quaternionToEuler(const geometry_msgs::Quaternion& quaternion) {
        tf2::Quaternion tf_quaternion;
        tf2::fromMsg(quaternion, tf_quaternion);

        tf2::Matrix3x3 tf_matrix(tf_quaternion);
        double roll, pitch, yaw;
        tf_matrix.getRPY(roll, pitch, yaw);

        return Eigen::Vector3d(roll, pitch, yaw);
    }

    // Callbacks
    void _imu_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
        // Implement IMU callback
        current_attitude_ = *msg;

        update_controller();
    }

    void _motors_vel_callback(const custom_msgs::Motors_vel::ConstPtr& msg) {
        // Implement motors velocity callback
        motors_vel_ = *msg;
    }

    void _odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Implement odometry callback
        ref_odometry_ = *msg;
    }

    // Publish motors output
    void publish_motors_output(const Eigen::VectorXd& output) {
        Eigen::VectorXd motors_vel_vectors = wheel_vel_distribution_ * output / factor_rpmtomps_;
        custom_msgs::Motors_vel motors_vel_msg;
        motors_vel_msg.motor1_rpm = motors_vel_vectors[0];
        motors_vel_msg.motor2_rpm = motors_vel_vectors[1];
        motors_vel_msg.motor3_rpm = motors_vel_vectors[2];
        motors_vel_msg.motor4_rpm = motors_vel_vectors[3];

        guide_motors_vel_pub_.publish(motors_vel_msg);
    }

    void update_controller() {
        if (!_is_init) {
            return;
        }

        current_ref_vel = ref_odometry_.twist.twist.linear.x;
        Eigen::Vector3d euler_angles = quaternionToEuler(ref_odometry_.pose.pose.orientation);
        current_ref_psi = euler_angles(2);

        // MPC controller update
        opti.set_value(X0_psi, current_attitude_.vector.z * 180.0 / M_PI);
        opti.set_value(ref_psi, current_ref_psi);
        opti.set_value(ref_vel, current_ref_vel);

        casadi::OptiSol sol = opti.solve();
        casadi::DM u_value = sol.value(U(all, 0));
        Eigen::VectorXd u_vector = Eigen::Map<Eigen::VectorXd>(u_value.ptr(), u_value.rows(), u_value.columns());
        // Eigen::VectorXd wheels_output = casadi::DMVectorToEigen(sol.value(U(all, 0))).transpose();

        publish_motors_output(u_vector);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller_node");
    std::cout << "MPCController start" << std::endl;
    ros::NodeHandle nodehandle;

    try {
        double wheel_diameter = 0; // Initialize with appropriate values
        double wheel_separation = 0; // Initialize with appropriate values
        MPCController controller(nodehandle, wheel_diameter, wheel_separation);
        ros::spin();
    } catch (const ros::Exception& e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }

    std::cout << "Exiting" << std::endl;
    return 0;
}
