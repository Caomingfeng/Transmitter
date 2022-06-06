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

    //TCP client setup
    //1.创建TCP socket，流式套接字
    // int socket_fd = socket(AF_INET, SOCK_STREAM,0);
    // if(socket_fd == -1)
    // {
    //     cout<<"socket 创建失败："<<endl;
    //     exit(-1);
    // }
    // uint8_t rx_msg[BUFFER_SIZE];
    // int len;
    // int pos;
    // uint8_t SOF = 0x5a;

    // // 2. 链接服务端
    // struct sockaddr_in addr;
    // memset(&addr, 0, sizeof(addr));
    // addr.sin_family = AF_INET;
    // addr.sin_port = htons(SERVERPORT);
    // addr.sin_addr.s_addr = inet_addr(HOST_IP);
    // cout<<"1："<< endl;
    // int res = connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr) != 0);
    // cout<<"2;"<< endl;
    // if(res == -1)
    // {
    //     cout<<"bind 链接失败："<< endl;
    //     exit(-1);
    // }
    // cout << "bind 链接成功：" << endl;
    // int sock;
    // char opmsg[13];
    // char get_msg[13] = {0};
    // int result, opnd_cnt, i;
    // int len;
    // bool if_first = true;
    // struct sockaddr_in serv_addr;
 
    // sock = socket(PF_INET, SOCK_STREAM, 0);
    // if(sock == -1){
    //     return -1;
    // }
    // memset(&serv_addr, 0, sizeof(serv_addr));
    // serv_addr.sin_family = AF_INET;
    // serv_addr.sin_addr.s_addr = inet_addr(HOST_IP);  // 注释1
    // serv_addr.sin_port = htons(SERVERPORT);
    // if(bind(sock, (struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1){ // 注释2
    //     cout << "connect error\n";
    //     return -1;
    // }
    // else{
    //     cout << "connected ...\n" << endl;  //注释3
    // }

    // ros::Rate r(50);
    // int ret = 0;
    // // 3.接收发送消息
    // while(ros::ok())
	// {      
	// 		// 接收服务端的消息数组
	// 		char buf[12];
	// 		// ret = read(socket_fd, buf, 12); // 接收数据到buf,并获得接收的长度ret.
    //         // if(ret > 0){    
    //         //     cout << "recv_data: " << buf << endl;//打印接受到的数据           
    //         //     for(int i = 0; i < ret; i++)
    //         //     {
    //         //         if(strcmp(&buf[i], ",") == 0){
    //         //             pos = i;
    //         //         }
    //         //     }
    //         //     double linear_velocity = (int)(buf[0]-48)+(int)(buf[2]-48)*0.1+(int)(buf[3]-48)*0.01;
    //         //     double angular_velocity = (int)(buf[pos]-48)+(int)(buf[pos+2]-48)*0.1+(int)(buf[pos+3]-48)*0.01;
    //         //     cout<< "linear ,angular"<<linear_velocity<<"," << angular_velocity << endl;
    //         // }			
	// 		// 发送
	// 		//ret = write(socket_fd, "26.0,0.5,0.21", strlen("26.0,0.5,0.21"));//向服务器发送数据
	// 		ros::spinOnce(); //ROS消息回调处理函数
	// 		r.sleep(); 
	// 	}
		//close(socket_fd);
    return 0;
}

