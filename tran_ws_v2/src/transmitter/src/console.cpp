/*
 * console.cpp
 *
 * Created on: Mar 24, 2022 10:24
 * Description:
 * 实例化transmitter类并运行
 *9
 * Copyright (c) 2022 Mingfeng Cao (cmf)
 */
#include "transmitter/transmitter.hpp"


using namespace std;

int main(int argc, char** argv)
{
    //ROS initial
    ros::init(argc, argv, "console");
    ros::NodeHandle n("");
    //char host_ip[12];
    string host_ip = "192.168.1.5";
    double pose[2];
    double resolution = 0;
    int my_id = 1;
    vector<double> p;
    n.getParam("/host_ip", host_ip);
    n.getParam("/resolution", resolution);
    n.getParam("/origin", p);
    n.getParam("/origin", pose[1]);
    n.getParam("/my_id", my_id);
    // ROS_INFO_STREAM("Get host_ip:"<<host_ip);
    // ROS_INFO_STREAM("Get resolution:"<<resolution);
    // ROS_INFO_STREAM("Get pose:"<< p[0] << ", " << p[1]);

    std::string device_name = "/dev/ttyUSB_AOA";
    int32_t baud_rate = 115200;
    Transmitter transmitter_(&n);
    //transmitter_.Open_Serial(device_name, baud_rate);
    transmitter_.SetTCP_para(host_ip);
    transmitter_.SetMap_para(p[0], p[1], resolution);
    transmitter_.SetMyID(my_id);
    transmitter_.Run();

    return 0;
}

