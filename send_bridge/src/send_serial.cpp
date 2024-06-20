#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <serial/serial.h>
#include <string>
#include <custom_msgs/Motors_vel.h>
#include <custom_msgs/Thruster_pwm.h>

serial::Serial ser;

void readFromSerial(ros::Publisher& speed_pub) {
    if (ser.available()) {
        custom_msgs::Motors_vel msg;
        std::string line = ser.readline();
        try {
            float motor_speed = std::stof(line);
            msg.motor1_rpm = motor_speed;
            msg.motor2_rpm = motor_speed;
            msg.motor3_rpm = motor_speed;
            msg.motor4_rpm = motor_speed;
            speed_pub.publish(msg);
        } catch (const std::invalid_argument& e) {
            ROS_ERROR("Invalid motor speed received: %s", line.c_str());
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_serial");
    ros::NodeHandle nh;
    ros::Publisher speed_pub = nh.advertise<custom_msgs::Motors_vel>("guide_Motors_vel", 10);

    std::string port = "/dev/ttyS1"; // 修改为实际的串口设备
    unsigned int baud_rate = 115200; // 修改为实际的波特率

    try {
        ser.setPort(port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        readFromSerial(speed_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
