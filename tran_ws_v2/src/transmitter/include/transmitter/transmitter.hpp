/*
 * transmitter.hpp
 *
 * Created on: Mar 24, 2022 10:24
 * Description:
 * 包含了
 * 主线程：订阅小车状态和里程计信息并处理
 * tcp连接线程：与指挥端互通状态和控制信息
 * AOA串口线程：根据收到的tag坐标实现跟踪
 * cmd control线程：根据当前控制循环发布cmd_vel信息
 *
 * Copyright (c) 2022 Mingfeng Cao (cmf)
 */

#ifndef TRANSMITTER_HPP
#define TRANSMITTER_HPP

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
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/transform_listener.h>

#define HOST_IP "192.168.1.6"
#define SERVERPORT_VIDEO 8010
#define SERVERPORT 8010
#define MOTION_HOST_IP "192.168.1.120"
#define MOTIONPORT 43893
#define FILEPORT 8020
#define BUFFER_SIZE 128
#define MIN_FRAME_LEN 12
#define FILEPATH "/home/ysc/perception_map/jueying.pgm"
#define SAVEPATH "/home/ysc/perception_map/new.pgm"
//using namespace cv;
//namespace 
class Transmitter
{
public:
  //构造函数和析够函数
  Transmitter(ros::NodeHandle *nh)
      : nh_(nh) {}
  ~Transmitter();

  //功能：设置服务器IP
  void SetTCP_para(std::string ip_);

  //功能：设置原始地图信息，包括左下角坐标和分辨率
  void SetMap_para(double x, double y, double res);
  void SetRobot_para(int id_, int aoa_enable_);

  //功能：初始化ros订阅器和发布器
  void SetupSubscription();
  
  //功能:初始化所有ros订阅器和发布器，开始所有线程
  void Run();

  //功能:已对应波特率打开对应串口
  void Open_Serial(std::string dev_name, int32_t baud_rate);

  //功能:终止时停止所有线程的循环，关闭串口
  void Terminate();

private:
  ros::NodeHandle *nh_;
  int mode = -1; //control mode
  //uint8_t my_id = '1';
  int my_id = 1;
  uint8_t my_team = 'x';
  int my_team_id = -1;
  bool first_go = false;
  bool in_team = false;
  bool receive_leader = false;
  geometry_msgs::PointStamped leader_pos;

  //所有线程
  std::thread tcp_thread_;
  std::thread file_tcp_thread_;
  std::thread cmd_thread_;
  std::thread back_thread_;
  std::thread serial_thread_;

  //线程循环的间隔
  int32_t tcp_thread_period_ms_ = 20;
  int32_t cmd_thread_period_ms_ = 40;
  int32_t back_thread_period_ms_ = 50;
  int32_t serial_thread_period_ms_ = 40;

  //线程开启标志位
  bool tcp_thread_started_ = false;
  bool cmd_thread_started_ = false;
  bool back_thread_started_ = false;
  bool serial_thread_started_ = false;

  //控制线程循环结束
  bool tcp_keep_running_ = false;
  bool file_tcp_keep_running_ = false;
  bool cmd_keep_running_ = false;
  bool back_keep_running_ = false;
  bool serial_keep_running_ = false;

  // tcp和udp连接相关参数
  int tcp_socket_;
  int udp_socket_ = -1;
  int file_tcp_socket_;

  struct sockaddr_in addr_;
  struct sockaddr_in udp_addr_;
  struct sockaddr_in file_addr_;

  char host_ip_[16];

  bool tcp_socket_used_flag = false;
  bool tcp_connected_flag_ = false;

  //用于发布当前控制速度
  int current_linear_speed = 0;
  int current_angular_speed = 0;
  int current_translation_speed = 0;
  int next_linear_speed = 0;
  int next_angular_speed = 0;
  int next_translation_speed = 0;

  //串口参数
  serial::Serial serial_;
  std::string device_name = "/dev/ttyUSB0";
  int32_t baud_rate = 115200;

  // 判断AOA信标的开启状态
  bool tag_open_flag = false;
  int aoa_enable = 0;

  //ros的发布器和订阅器
  ros::Publisher initial_pose_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher cancel_pub_;
  ros::Publisher motion_cmd_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber motor_tem_sub_;
  ros::Subscriber driver_tem_sub_;
  ros::Subscriber robot_basic_state_sub_;
  ros::Subscriber robot_gait_state_sub_;
  ros::Subscriber control_mode_sub_;
  ros::Subscriber status_sub_;

  //tf监听，用于进行点的坐标系变换
  tf::TransformListener listener;

  //巡逻模式下的点的存储以及各种参数
  std::vector<geometry_msgs::PointStamped> goal_vec;
  actionlib_msgs::GoalID cur_goalid_;
  actionlib_msgs::GoalID last_goalid_;
  int goal_status = -1;
  int goal_count = 0;
  bool circle_flag = false;
  bool nav_done = true;
  bool first_nav = true;
  bool continue_flag = true;

  //无人狗状态信息
  double battery = 0;
  int robot_gait = 0;
  int robot_base = 0;
  int control_mode = 0 ;
  double pos_x = 0, pos_y = 0;
  double pos_angle = 0;
  int pix_x = 0, pix_y = 0; //以左下角为坐标原点的当前位置的像素坐标
  double origin_x = 0;
  double origin_y = 0; //地图最左下角的坐标值
  double resolution = 0.1;

  //跟随模式时上一次和下一次跟随目标相对与本机的x，y坐标
  int current_x_position;
  int current_y_position;
  int next_x_position;
  int next_y_position;

  //线程锁，防止串口线程频繁改变跟随位置时，cmd线程读取错误
  std::mutex pos_mutex;

  geometry_msgs::Twist current_twist_cmd_;
  geometry_msgs::Twist next_twist_cmd_;
  int static_count = 0;
  int t_count = 0;
  bool static_flag = false; //flag to justice if the car is stable
  int tag_open_count = 0;
  int point_count = 0;

  //跟随的x，y方向距离，x代表前后，y为左右
  double dis_x = 200;
  double dis_y = 0;
  double dis_man = 120;
  //大于设定的最大跟随距离
  double max_x = 400;
  double max_y = 250;
  //小于设定的最大靠近距离
  double min_x = 200;
  double min_y = 0;

  bool angular_close = false;

  double max_linear_speed = 1.7;    //max linear speed of the car
  double max_angle_speed = 0.61;   //max angular speed of the car
  // int co_x = 0, co_y =0;

  ros::Time last_time_;
  ros::Time current_time_;
  //无人狗udp协议，简单指令
  struct CommandHead
  {
    uint32_t code;
    uint32_t paramters_size;
    //  This value indicate whether you have messages to be send behind the
    // address of the value named type
    //  @param 1 : There are messages to be send
    //  @param 0 :There is nothing to be send ;
    uint32_t type;
  };
  const uint32_t kDataSize = 256;
  //无人狗udp协议，复杂指令
  struct Command
  {
    CommandHead head;
    //uint8_t data[256];
    int data;
  };

  //功能：初始化服务器tcp和运动主机udp的scoket
  void Tcp_setup();

  //发起Tcp连接
  int Tcp_connect();

  //第五回传线程，将本地地图发回到指挥端
  void StartFileTcpThread();

  // TCP线程，循环判断tcp是否连接
  void FileTcpLoop();

  //初始化tcp和udp的scoket，其中tcp连接用于和指挥端通信，udp连接用于向运动主机发送控制信息
  void FileTcp_setup();

  //尝试tcp连接
  int FileTcp_connect();

  //开启tcp线程
  void StartTcpThread();

  // TCP线程，判断tcp是否连接进行自动重连;接受指挥端信息进行处理
  void TcpLoop(int32_t period_ms);

  //开启控制线程，手动模式下udp发送速度;跟随模式下跟随AOA信标;巡逻模式下循环发布已保存的目标点
  void StartCmdThread();

  //控制线程:，手动模式下udp发送速度;
  //跟随模式下跟随AOA信标;
  //巡逻模式下循环发布已保存的目标点
  void CmdLoop(int32_t period_ms);

  //以无人狗协议UDP发送当前的速度到运动主机
  void SendSpeed();

  //跟随当前的目标位置
  void FollowControl();
 
  //无人狗状态返回线程
  void StartBackThread();

  // 发送无人狗状态信息至指挥端
  void BackLoop(int32_t period_ms);

  //开启串口循环线程
  void StartSerialThread();

  //循环读取串口的数据，根据aoa基站协议解析
  void SerialLoop(int32_t period_ms);

  //根据AOA基站的数据协议，解析当前tag的X和Y坐标
  void ParseUARTBuffer(uint8_t *buf, int bytes_received);

  //无人狗简单指令的协议，指令码+指令值
  void SendSimpleUDP(int code_, uint32_t size);

  //complex udp command,contain code and code value
  void SendComplexUDP(int code_, int32_t value_);

  //odom话题回调函数，无人狗上odom即为在map中的坐标
  //转换为地图像素坐标和yaw角后保存
  void OdomCallBack(const nav_msgs::Odometry::ConstPtr &msg);

  // void ImuCallBack(const sensor_msgs::Imu::ConstPtr &msg)
  // {
  // }
  //无人狗电量百分比
  void BatteryCallBack(const std_msgs::Float32::ConstPtr &msg);

  void MotortemCallBack(const std_msgs::Float32::ConstPtr &msg);

  void DrivertemCallBack(const std_msgs::Float32::ConstPtr &msg);

  //无人狗当前状态(趴下，站立，力控，踏步站立等)
  void BasicStateCallBack(const std_msgs::Int32::ConstPtr &msg);

  //无人狗步态
  void GaitStateCallBack(const std_msgs::Int32::ConstPtr &msg);

  //无人狗控制模式(手动，导航)
  void ControlModeCallBack(const std_msgs::Int32::ConstPtr &msg);

  //导航点状态 1正在前往 3已到达
  void GoalStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses);

  //将map中的一个点坐标转换到baselink坐标系
  void Point_Map2Base(double x_pos, double y_pos);

  //将map中的一个点坐标转换到baselink坐标系
  void Point_Base2Map(int x_pos1, int y_pos1);

  //发布当前点为导航点，方向设定为本机指向目标点的角度
  void Publish_Goal(double nav_x, double nav_y);

  //发布当前点为导航点，方向设定为本机指向目标点的角度
  void Publish_Goal_Co(double nav_x, double nav_y, double nav_angle);

  //取消当前导航点
  void CancelNavi();

  //参数含义:Kalman_KX 卡尔曼系数  Kalman_XD 当前估计值 Kalman_ZX  上一次测量值
  //Kalman_ESTX 估计误差   Kalman_MEAX 测量误差
  void Kalman(double x_position, double y_position);

  // 根据当前跟随目标相对与本机的x，y位置，对比设定的跟随距离以及最大跟随距离，
  // 计算相应的线速度和角速度；在转弯时降低线速度
  void follow(int x_position, int y_position);

  //根据协同跟随初始化信息，计算本机处于队形中的位置，发布导航点
  void CoTeam(geometry_msgs::PointStamped *leader, uint8_t team_code, int team_id);

};
// namespace westonrobot

#endif /* TRANSMITTER_HPP */
