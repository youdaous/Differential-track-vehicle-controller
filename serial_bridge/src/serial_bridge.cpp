#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <iomanip>
#include <geometry_msgs/Vector3Stamped.h>
#include <custom_msgs/Motors_vel.h>
#include <custom_msgs/Thruster_pwm.h>
#include <custom_msgs/HeightDepth.h>

//定义一个标志位，表示是否开始存储字节到数组中
bool startStoring = false;

//存储接收数据的索引
int buffer_shou_index = 0;

//接收的数据是否正确
bool jieshou_true = false;

//第一个目标距离与回波电平
float first_target_dist,first_Electriclevel;
//最强目标距离与回波电平
float stong_target_dist,stong_Electriclevel;
//最后一个目标距离与回波电平
float last_target_dist,last_Electriclevel;

//存储姿态 度 显示用
float head;
float pitch;
float roll;

//转速读取 显示用
int PBL70_speed1;
int PBL70_speed2;
int PBL70_speed3;
int PBL70_speed4;

//发送期望速度值
int target_speed1 = 0;
int target_speed2 = 0;
int target_speed3 = 0;
int target_speed4 = 0;

//接收深度计的数值
int pressure = 0;
float temperature = 0;

// 垂推推进器pwm
int pwm1 = 30,pwm2 =80;

//发送的50个字节
#define buffer_fa_geshu  50
uint8_t buffer_fa[buffer_fa_geshu];    

//接收的buffer_shou_geshu个字节
#define buffer_shou_geshu 60
uint8_t buffer_shou[buffer_shou_geshu]; 

//异或校验
uint8_t CheckXOR(uint8_t *Buf, uint8_t Len)
{
    uint8_t x = 0;
    for (uint8_t i = 0; i < Len; i++) 
    {
        x = x ^ (*(Buf + i));
    }
    return x;
}

//将接收到的四位速度值转换为十进制数
int speed_u8_int(const unsigned char* hexArray) 
{
    int result = 0;

    // Combine bytes into an integer
    result |= hexArray[0];                      // Low byte
    result |= (hexArray[1] << 8);
    result |= (hexArray[2] << 16);
    result |= (hexArray[3] << 24);              // High byte

    return result;
}

//int类型的速度值转换为低位在前高位在后的四个u值
void speed_int_u8(int speed_value, uint8_t* hex_array) 
{
    // 将整数速度值转换为无符号整数类型
    unsigned int hexValue = (unsigned int)speed_value;
    
    // 拆分十六进制值为四个字节
    hex_array[0] = (hexValue & 0xFF);           // 低字节
    hex_array[1] = ((hexValue >> 8) & 0xFF);
    hex_array[2] = ((hexValue >> 16) & 0xFF);
    hex_array[3] = ((hexValue >> 24) & 0xFF);   // 高字节
}


//读取姿态角的数值
float IMU_calculate(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) 
{
    // 将四个字节按照大端序顺序拼接成一个 32 位整数
    uint32_t floatBits = (static_cast<uint32_t>(byte1) << 24) |
	                     (static_cast<uint32_t>(byte2) << 16) |
	                     (static_cast<uint32_t>(byte3) << 8)  |
	                      static_cast<uint32_t>(byte4);
     // 提取符号位
    int S = (floatBits >> 31) & 0x1;

    // 提取指数部分
    int exponent = (floatBits >> 23) & 0xFF;

    // 提取尾数部分
    int mantissa = floatBits & 0x7FFFFF;

    // 计算浮点数的值
    float value = 0.0f;
    if (exponent != 0 && exponent != 255) 
    {
        value = pow(-1, S) * pow(2, exponent - 127) * (1 + (mantissa / pow(2, 23)));
    }
    
    return value;
}


//高度计的计算方式
float Altimeter_calculate(uint8_t byte1, uint8_t byte2) 
{
    // 将两个字节合并为一个 16 位整数
    uint16_t combined = (uint16_t)(byte1) * 256 + (uint16_t)(byte2);
    // 将 16 位整数解释为 float 类型
    return (float)combined;
}


//深度计计算
int Depthometer_pressure(uint8_t high_byte, uint8_t low_byte) 
{
	//将两个字节合并成一个，然后转换为int类型
    uint16_t value=(static_cast<uint16_t>(high_byte) << 8) | low_byte;
    int int_value = static_cast<int>(value);

    int pressure = (int_value - 16384) * (100 - 0) / 32768;

    return pressure;
}
float Depthometer_temperature(uint8_t high_byte, uint8_t low_byte) 
{
	//将两个字节合并成一个，然后转换为int类型
    uint16_t value=(static_cast<uint16_t>(high_byte) << 8) | low_byte;
    int int_value = static_cast<int>(value);

	float temperature = (int_value - 384) * 0.003125 - 50;

    return temperature;
}


bool openSerialPort(serial::Serial &sp, const std::string &portName, uint32_t baudrate) //判断串口
{
	serial::Timeout to = serial::Timeout::simpleTimeout(100);
	sp.setPort(portName);
	sp.setBaudrate(baudrate);
	sp.setTimeout(to);
	try 
	{
		sp.open();
	} 
	catch (serial::IOException &e) 
	{
		return false;
	}

	if (sp.isOpen()) 
	{
		return true;
	} 
	else 
	{
		return false;
	}
}

void guideMotorsCallback(const custom_msgs::Motors_vel msg)
{
    target_speed1 = msg.motor1_rpm;
    target_speed2 = msg.motor2_rpm;
    target_speed3 = msg.motor3_rpm;
    target_speed4 = msg.motor4_rpm;
}

void guideThrusterCallbck(const custom_msgs::Thruster_pwm msg)
{
	pwm1 = msg.pwm1;
	pwm2 = msg.pwm2;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "rov_TTL");
	ros::NodeHandle n;
    // 发布小车姿态欧拉角和电机转速
    ros::Publisher attitudePub = n.advertise<geometry_msgs::Vector3Stamped>("attitude_inEuler", 1);
    ros::Publisher motors_velPub = n.advertise<custom_msgs::Motors_vel>("motors_vel_inrpm", 1);
	ros::Publisher heightDepthPub = n.advertise<custom_msgs::HeightDepth>("Height_Depth", 1);
    // 订阅电机转速指令
    ros::Subscriber guideMotors_velSub = n.subscribe<custom_msgs::Motors_vel>("guide_Motors_vel", 1, guideMotorsCallback);
	// 订阅垂推推进器指令
	ros::Subscriber thrusters_pwmSub = n.subscribe<custom_msgs::Thruster_pwm>("guide_Thrusters_pwm", 1, guideThrusterCallbck);

	ros::Rate loop_rate(100);

	serial::Serial sp;

	const std::vector<std::string> portNames = {"/dev/ttyUSB0"};

	uint8_t jishu=0;//范围0到255  累加计数

	//循环连接串口
	while (ros::ok()) 
	{
	 	for (const std::string &portName : portNames) 
	 	{
	 		if (openSerialPort(sp, portName, 115200)) 
	 		{
	 			ROS_INFO_STREAM("Open ready: " << portName);
	 			break;
	 		}
	 		ros::Duration(1.0).sleep();
	 	}

	 	if (sp.isOpen()) 
	 	{
	 		break;
	 	}
	 	loop_rate.sleep();
	}

	while (ros::ok())
	{
	   //接收数据//
	 	size_t n = sp.available();
		//ROS_INFO("n: %d\n ", n);

	 	if (n >= 100) // 有数据接收
		{
		    n = sp.read(buffer_shou, n);
		    ROS_INFO("n: %zu\n ", n);
		    sp.flushInput();//清楚接收数据缓存区，保证数据的实时性

		    for (size_t i = 0; i < n - buffer_shou_geshu; ++i)
		    {
		        if (buffer_shou[i]                         == 0xAB && 
		        	buffer_shou[i + 1]                     == 0x14 &&
		            buffer_shou[i + buffer_shou_geshu - 2] == 0x5A && 
		            buffer_shou[i + buffer_shou_geshu - 1] == 0x5A
		            )
		        {
		            // 将从0xAB到0x5A之间的五十个字节重新存储到buffer_shou中
		            for (size_t j = 0; j < buffer_shou_geshu; ++j)
		            {
		                buffer_shou[j] = buffer_shou[i + j];
		                jieshou_true = true;
		            }
		            break;
		        }
    		}
		    // 判断是否已经存储满buffer_shou_geshu个字节
		    
	        //判断前两个字节是否为0xAB和0x14 判断最后两个字节是否为0x5A和0x5A
	        if(jieshou_true)
	        {
	        	//接收的数据中倒数第三个异或校验的结果
	        	uint8_t check1;
	        	check1 = CheckXOR(buffer_shou,buffer_shou_geshu - 3); 

		        if (buffer_shou[0]                   == 0xAB && 
		        	buffer_shou[1]                   == 0x14 && 
		        	buffer_shou[buffer_shou_geshu-2] == 0x5A && 
		        	buffer_shou[buffer_shou_geshu-1] == 0x5A &&
		        	buffer_shou[buffer_shou_geshu-3] == check1 
		        	) 
		        {
		            // 打印存储的字节
		            std::cout << "Received bytes: ";
		            for (int i = 0; i < buffer_shou_geshu; i++) 
		            {
		                std::cout  << std::dec << i <<": "<< "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer_shou[i]) << " ";
		            }
		            std::cout << std::dec << std::endl;

		            //数据将先读取低八位后读取高八位，将这两个字节合并成转速值
					//电机1转速
					unsigned char hexArray1[4];
					hexArray1[0] = buffer_shou[3];// Low byte
				    hexArray1[1] = buffer_shou[4];
				    hexArray1[2] = buffer_shou[5];
				    hexArray1[3] = buffer_shou[6];// High byte
					PBL70_speed1 = speed_u8_int(hexArray1);
					//电机2转速
					unsigned char hexArray2[4];
					hexArray2[0] = buffer_shou[7];// Low byte
				    hexArray2[1] = buffer_shou[8];
				    hexArray2[2] = buffer_shou[9];
				    hexArray2[3] = buffer_shou[10];// High byte
					PBL70_speed2 = speed_u8_int(hexArray2);
					//电机3转速
					unsigned char hexArray3[4];
					hexArray3[0] = buffer_shou[11];// Low byte
				    hexArray3[1] = buffer_shou[12];
				    hexArray3[2] = buffer_shou[13];
				    hexArray3[3] = buffer_shou[14];// High byte
					PBL70_speed3 = speed_u8_int(hexArray3);
					//电机4转速
					unsigned char hexArray4[4];
					hexArray4[0] = buffer_shou[15];// Low byte
				    hexArray4[1] = buffer_shou[16];
				    hexArray4[2] = buffer_shou[17];
				    hexArray4[3] = buffer_shou[18];// High byte
					PBL70_speed4 = speed_u8_int(hexArray4);

					//计算姿态角
					head  = IMU_calculate(buffer_shou[37],buffer_shou[38],buffer_shou[39],buffer_shou[40]);
					pitch = IMU_calculate(buffer_shou[42],buffer_shou[43],buffer_shou[44],buffer_shou[45]);
					roll  = IMU_calculate(buffer_shou[47],buffer_shou[48],buffer_shou[49],buffer_shou[50]);

					//计算高度计的值 单位10mm
					first_target_dist   = Altimeter_calculate(buffer_shou[24],buffer_shou[25]);
					first_Electriclevel = Altimeter_calculate(buffer_shou[26],buffer_shou[27]);
					stong_target_dist   = Altimeter_calculate(buffer_shou[28],buffer_shou[29]);
					stong_Electriclevel = Altimeter_calculate(buffer_shou[30],buffer_shou[31]);
					last_target_dist    = Altimeter_calculate(buffer_shou[32],buffer_shou[33]);
					last_Electriclevel  = Altimeter_calculate(buffer_shou[34],buffer_shou[35]);

					//计算深度计数值
					pressure    = Depthometer_pressure(buffer_shou[20],buffer_shou[21]);
					temperature = Depthometer_temperature(buffer_shou[22],buffer_shou[23]);

					//显示期望的的转速
					printf("\ntarget_speed1: %d target_speed2: %d target_speed3: %d target_speed4: %d\n  ", target_speed1,target_speed2,target_speed3,target_speed4);
					//显示读取的转速
					printf("\nspeed1: %d speed2: %d speed3: %d speed4: %d\n\n  ", PBL70_speed1,PBL70_speed2 ,PBL70_speed3,PBL70_speed4);
					//显示姿态角
					printf("Heading: %.3f Pitch: %.3f Roll: %.3f\n\n ", head, pitch, roll);
					//高度计显示
					printf("first_target_dist: %.1fmm, first_Electriclevel: %.1fv\n stong_target_dist: %.1fmm, stong_Electriclevel: %.1fv\n last_target_dist: %.1fmm,  last_Electriclevel: %.1fv\n\n", 
							first_target_dist, first_Electriclevel, stong_target_dist,stong_Electriclevel,last_target_dist ,last_Electriclevel);
					//深度计
					printf("pressure: %d bar, temperature: %.3f ℃\n", pressure, temperature);

					//漏水检测
					if(buffer_shou[51]==0x02)
					{
						printf("value: %d leak: No\n", buffer_shou[51]);
					}
					if(buffer_shou[51]==0x01)
					{
						printf("value: %d leak: Yes\n", buffer_shou[51]);
					}

					jieshou_true = false;
					sp.flushInput();
		        } 
		        else 
		        {
		            std::cout << "Reception failed. Restarting..." << std::endl;
					jieshou_true = false;
		           	sp.flushInput();
		        }   
		    }
		}

		//发送数据//
		uint8_t check2;
	        	check2 = CheckXOR(buffer_fa,buffer_fa_geshu - 3); 
		buffer_fa[0]                = 0xAB;
		buffer_fa[1]                = 0X14;
		buffer_fa[2]                = buffer_fa_geshu;
		buffer_fa[buffer_fa_geshu-3]= check2;
		buffer_fa[buffer_fa_geshu-2]= 0x5A;
		buffer_fa[buffer_fa_geshu-1]= 0x5A;

		//设置四个电机的期望速度
		target_speed1 = 500;
		target_speed2 = 500;
		target_speed3 = -500;
		target_speed4 = -500;

		uint8_t target_speed1_u8[4];
		uint8_t target_speed2_u8[4];
		uint8_t target_speed3_u8[4];
		uint8_t target_speed4_u8[4];

		//电机1期望转速发送的低高八位数据
		speed_int_u8(target_speed1,target_speed1_u8);
	   	buffer_fa[3]  = target_speed1_u8[0];    
    	buffer_fa[4]  = target_speed1_u8[1];
	   	buffer_fa[5]  = target_speed1_u8[2];       
    	buffer_fa[6]  = target_speed1_u8[3];

    	//电机2期望转速发送的低高八位数据
    	speed_int_u8(target_speed2,target_speed2_u8);
	   	buffer_fa[7]  = target_speed2_u8[0];    
    	buffer_fa[8]  = target_speed2_u8[1];
	   	buffer_fa[9]  = target_speed2_u8[2];       
    	buffer_fa[10] = target_speed2_u8[3];

    	//电机3期望转速发送的低高八位数据
    	speed_int_u8(target_speed3,target_speed3_u8);
	   	buffer_fa[11] = target_speed3_u8[0];    
    	buffer_fa[12] = target_speed3_u8[1];
	   	buffer_fa[13] = target_speed3_u8[2];       
    	buffer_fa[14] = target_speed3_u8[3];

    	//电机4期望转速发送的低高八位数据
    	speed_int_u8(target_speed4,target_speed4_u8);
	   	buffer_fa[15] = target_speed4_u8[0];    
    	buffer_fa[16] = target_speed4_u8[1];
	   	buffer_fa[17] = target_speed4_u8[2];       
    	buffer_fa[18] = target_speed4_u8[3];

    	// int pwm1 = 30,pwm2 =80;
    	buffer_fa[19] = pwm1;
    	buffer_fa[20] = pwm2;

		// uint8_t he = 0; //发送校验和
		// for(int i=4; i<49; i++)
		// { 
		// 	he = he + buffer_fa[i];
		// }   
		// buffer_fa[49]=he& 0xFF;

		//发送数据 根据需要调整大小
		sp.write(buffer_fa, buffer_fa_geshu);  

		// std::cout << "Send bytes: ";
		// for (int i = 0; i < buffer_fa_geshu; i++) 
        // {
        //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') 
        //     << static_cast<int>(buffer_fa[i]) << " ";
        // }
        // std::cout << std::dec << std::endl;
        // sp.flushOutput();

		// 发布ros消息
        geometry_msgs::Vector3Stamped attitude_inEuler;
        custom_msgs::Motors_vel motors_rpm;
		custom_msgs::HeightDepth height_depth_data;
        ros::Time current_time = ros::Time::now();
        attitude_inEuler.header.stamp = current_time;
        attitude_inEuler.vector.x = roll;
        attitude_inEuler.vector.y = pitch;
        attitude_inEuler.vector.z = head;

        motors_rpm.header.stamp = current_time;
        motors_rpm.motor1_rpm = PBL70_speed1;
        motors_rpm.motor2_rpm = PBL70_speed2;
        motors_rpm.motor3_rpm = PBL70_speed3;
        motors_rpm.motor4_rpm = PBL70_speed4;

		height_depth_data.header.stamp = current_time;
		height_depth_data.first_target_dist = first_target_dist;
		height_depth_data.strong_target_dist = stong_target_dist;
		height_depth_data.last_target_dist = last_target_dist;
		height_depth_data.pressure = pressure;
		height_depth_data.temperature = temperature;

        motors_velPub.publish(motors_rpm);
		attitudePub.publish(attitude_inEuler);
		heightDepthPub.publish(height_depth_data);
		ros::spinOnce();
		loop_rate.sleep();
	}

	sp.close();
	return 0;
}
