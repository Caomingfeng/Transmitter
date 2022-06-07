#include <stdio.h>
#include <string.h>
#include <string>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mutex>
#include <iostream>
#include <thread>
#include <serial/serial.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

serial::Serial serial_;
geometry_msgs::Point point;

//参数含义:Kalman_KX 卡尔曼系数  Kalman_XD 当前估计值 Kalman_ZX  上一次测量值
//Kalman_ESTX 估计误差   Kalman_MEAX 测量误差
void Kalman(double x_position, double y_position)
{
        static double Kalman_KX, Kalman_XA, Kalman_ZA, Kalman_ESTA = 5, Kalman_MEAA = 3;
        static double Kalman_KY, Kalman_XD, Kalman_ZD, Kalman_ESTD = 5, Kalman_MEAD = 3; //

        Kalman_MEAA = fabs(Kalman_ZA - x_position);
        Kalman_KX = Kalman_ESTA / (Kalman_ESTA + Kalman_MEAA);
        Kalman_XA = Kalman_XA + Kalman_KX * (x_position - Kalman_XA);

        Kalman_MEAD = fabs(Kalman_ZD - y_position);
        Kalman_KY = Kalman_ESTD / (Kalman_ESTD + Kalman_MEAD);
        Kalman_XD = Kalman_XD + Kalman_KY * (y_position - Kalman_XD);
        //	//更新
        Kalman_ZA = x_position; //为下一次滤波做准备
        Kalman_ZD = y_position;

        point.x = Kalman_XA;
        point.y = Kalman_XD;
        //std::cout<<"before kalman position"<<x_position<<","<<y_position<<std::endl;
        //std::cout<<"Kalman position"<<next_x_position<<","<<next_y_position<<std::endl;
}

//根据UWB协议，解析当前标签相对于基站的坐标，进行卡尔慢滤波
void ParseUARTBuffer(uint8_t *buf, int bytes_received)
{
        int i, j, k;
        int recv_x = 0;
        int recv_y = 0;
        for (i = 0; i < bytes_received; ++i)
        {
                if (buf[i] == 'X' && buf[i + 1] == 'c' && buf[i + 2] == 'm')
                {
                        //position_mutex_.lock();
                        recv_x = 0;
                        uint8_t x_pos[10];
                        // std::cout << "detect Xcm" << std::endl;
                        for (j = 0; j < 10; j++)
                        {
                                if (buf[i + 5 + j] == ',')
                                {
                                        break;
                                }
                                else
                                {
                                        x_pos[j] = buf[i + 5 + j];
                                }
                        }
                        // std::cout << "j:" << j << std::endl;
                        if (x_pos[0] == '-')
                        {
                                for (k = j - 1; k >= 1; k--)
                                {
                                        recv_x += (int)(x_pos[k] - 48) * pow(10, j - 1 - k);
                                }
                                recv_x = -recv_x;
                        }
                        else
                        {
                                for (k = j - 1; k >= 0; k--)
                                {
                                        recv_x += (int)(x_pos[k] - 48) * pow(10, j - 1 - k);
                                }
                        }
                        //position_mutex_.unlock();
                        // int high = x_pos[1];
                        // int low = x_pos[0];
                        // std::cout << "Xcm:" << x_pos << std::endl;
                        //   std::cout << "Xcm:" << recv_x <<std::endl;
                }
                if (buf[i] == 'Y' && buf[i + 1] == 'c' && buf[i + 2] == 'm')
                {
                        //position_mutex_.lock();
                        // std::cout << "detect Ycm" << std::endl;
                        recv_y = 0;
                        uint8_t y_pos[10];
                        for (j = 0; j < 10; j++)
                        {
                                if (buf[i + 5 + j] == ',')
                                {
                                        break;
                                }
                                else
                                {
                                        y_pos[j] = buf[i + 5 + j];
                                }
                        }
                        if (y_pos[0] == '-')
                        {
                                for (k = j - 1; k >= 1; k--)
                                {
                                        recv_y += (int)(y_pos[k] - 48) * pow(10, j - 1 - k);
                                }
                                recv_y = -recv_y;
                        }
                        else
                        {
                                for (k = j - 1; k >= 0; k--)
                                {
                                        recv_y += (int)(y_pos[k] - 48) * pow(10, j - 1 - k);
                                }
                        }
                        //   std::cout << "Ycm" << y_pos <<std::endl;
                        // std::cout << "Ycm" << recv_y <<std::endl;
                }
        }
        //对收到的坐标进行卡尔曼滤波
        Kalman(recv_y, recv_x);
}

int main(int argc, char **argv)
{
        //ROS initial
        ros::init(argc, argv, "uwbdata");
        ros::NodeHandle n;
        ros::Publisher uwb_pub;
        uwb_pub = n.advertise<geometry_msgs::Point>(
            "/uwbpose", 5, true);

        std::string device_name = "/dev/ttyUSB0";
        int32_t baud_rate = 115200;

        serial_.setPort(device_name);
        serial_.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(to);
        try
        {
                /* code */
                serial_.open();
                std::cout << "AoA serial is opened!" << std::endl;
        }
        catch (const std::exception &e)
        {
                std::cerr << e.what() << '\n';
                exit(1);
        }

        while (ros::ok())
        {
                if (serial_.isOpen())
                {
                        size_t length = serial_.available();
                        if (length != 0)
                        {
                                //tag_open_flag = true;
                                uint8_t read_buffer[1024];
                                //防止读取buffer不够而出错
                                if (length > 1024)
                                {
                                        length = serial_.read(read_buffer, 1024);
                                }
                                else
                                {
                                        length = serial_.read(read_buffer, length);
                                }
                                // test, print the UART msg
                                // std::cout << read_buffer << std::endl;

                                //解析串口数据，提取tag的坐标
                                ParseUARTBuffer(read_buffer, length);
                                memset(read_buffer, '\0', sizeof(read_buffer));
                                uwb_pub.publish(point);
                                usleep(40000);
                        }
                }
        }
        return 0;
}
