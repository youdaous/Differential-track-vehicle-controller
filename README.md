# Environment
1. Ubuntu20.04
2. ROS noetic

# Dependience
1. Casadi
2. ros-noetic-serial
   
# Description for package
1. custom_msgs
   自定义消息类型，包括高度计、深度计、电机转速、推进器pwm值;
2. serial_bridge
   串口数据与ROS话题数据转发;
3. tracked_vehicle_controller
   定深定高-->script-->depth_mpc_controller.py
   水平面运动控制-->script-->mmpc_controller.py,
   **mmpc_controller.py是mpc_controller.py的替代, 其将轮上速度分配从mpc的运动学模型中分离**
   
