#include "transmitter/transmitter.hpp"

  /**************************************/
  //public function
  Transmitter::~Transmitter()
  {
    if (tcp_thread_.joinable())
    {
      tcp_thread_.detach();
    }
    if (back_thread_.joinable())
    {
      back_thread_.detach();
    }
    if (cmd_thread_.joinable())
    {
      cmd_thread_.detach();
    }
    if (serial_thread_.joinable())
    {
      serial_thread_.detach();
    }
  }
 //功能：设置服务器IP
  void Transmitter::SetTCP_para(std::string ip_)
  {
    strcpy(host_ip_, ip_.c_str());
    ROS_INFO("Set server IP: %s", host_ip_);
  }

    //功能：设置原始地图信息，包括左下角坐标和分辨率
  void Transmitter::SetMap_para(double x, double y, double res)
  {
    origin_x = x;
    origin_y = y;
    resolution = res;
    ROS_INFO("Map:origin_x:%f, origin_y:%f, resolution:%f", origin_x, origin_y, resolution);
  }

  //set robot ID
  void Transmitter::SetRobot_para(int id_, int aoa_flag_, int control_mode_)
  {
    my_id = id_;
    aoa_enable = aoa_flag_;
    mode = control_mode_;
     ROS_INFO("Set machine ID: %d, AoA enable: %d, Control mode: %d", my_id, aoa_enable, mode);
  }

  //功能：初始化ros订阅器和发布器
  void Transmitter::SetupSubscription()
  {
    // initial_pose_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, true);
    goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(
        "move_base_simple/goal", 5, true);
    cancel_pub_ = nh_->advertise<actionlib_msgs::GoalID>(
        "move_base/cancel", 1);
    motion_cmd_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    odom_sub_ = nh_->subscribe<nav_msgs::Odometry>(
        "/odom", 10, &Transmitter::OdomCallBack, this);
    // imu_sub_ = nh_->subscribe<sensor_msgs::Imu>(
    //     "/imu/data", 10, &Transmitter::ImuCallBack, this);
    battery_sub_ = nh_->subscribe<std_msgs::Float32>(
        "/battery_level", 10, &Transmitter::BatteryCallBack, this);
    motor_tem_sub_ = nh_->subscribe<std_msgs::Float32>(
        "/motor_temperature", 10, &Transmitter::MotortemCallBack, this);
    driver_tem_sub_ = nh_->subscribe<std_msgs::Float32>(
        "/driver_temperature", 10, &Transmitter::DrivertemCallBack, this);
    robot_basic_state_sub_ = nh_->subscribe<std_msgs::Int32>(
        "/robot_basic_state", 10, &Transmitter::BasicStateCallBack, this);
    robot_gait_state_sub_ = nh_->subscribe<std_msgs::Int32>(
        "/robot_gait_state", 10, &Transmitter::GaitStateCallBack, this);
    control_mode_sub_ = nh_->subscribe<std_msgs::Int32>(
        "/control_mode", 10, &Transmitter::ControlModeCallBack, this);
    status_sub_ = nh_->subscribe<actionlib_msgs::GoalStatusArray>(
        "move_base/status", 1, &Transmitter::GoalStatusCallBack, this);
  }

  //功能:初始化所有ros订阅器和发布器，开始所有线程
  void Transmitter::Run()
  {
    SetupSubscription();
    //StartVideoTcpThread();
    StartTcpThread();
    StartCmdThread();
    StartBackThread();
    if(aoa_enable){
      StartSerialThread();
    }  
    //StartFileTcpThread();

    // 50hz频率运行话题callback函数
    ros::Rate rate(50);
    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Main function terminated!");
    Terminate();
  }

    //功能:已对应波特率打开对应串口
  void Transmitter::Open_Serial(std::string dev_name, int32_t baud_rate)
  {
    serial_.setPort(dev_name);
    serial_.setBaudrate(baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(to);

    try
    {
      /* code */
      serial_.open();
      ROS_INFO("AoA serial is opened!");
    }
    catch (const std::exception &e)
    {
      ROS_WARN_STREAM(e.what() << '\n');
    }
    // start serial read
  }

    //功能:终止时停止所有线程的循环，关闭串口
  void Transmitter::Terminate()
  {
    ROS_INFO("Set keep running flags false!");
    tcp_keep_running_ = false;
    back_keep_running_ = false;
    cmd_keep_running_ = false;
    serial_keep_running_ = false;
    file_tcp_keep_running_ = false;
    if (serial_.isOpen())
    {
      serial_.close();
    }
    if(goal_status = 1){
       CancelNavi();
    }
    // std::terminate();
  }

  /**************************************/
  //private function

  //功能：初始化服务器tcp和运动主机udp的scoket
  void Transmitter::Tcp_setup()
  {
    tcp_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tcp_socket_ == -1)
    {
      ROS_WARN("TCP Socket create failed!");
    }
    tcp_socket_used_flag = false;
    //std::cout << "Video Socket create" <<tcp_socket_<< std::endl;
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(SERVERPORT);
    //addr_.sin_addr.s_addr = inet_addr(HOST_IP);
    addr_.sin_addr.s_addr = inet_addr(host_ip_);

    //udp套接字只创建一次
    if(udp_socket_ < 0){
      udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
      if (udp_socket_ == -1)
      {
        ROS_WARN("UDP Socket create failed!");
      }
      udp_addr_.sin_family = AF_INET;
      udp_addr_.sin_port = htons(MOTIONPORT);
      udp_addr_.sin_addr.s_addr = inet_addr(MOTION_HOST_IP);
    }   
  }

  //发起Tcp连接
  int Transmitter::Tcp_connect()
  {
    int res_tcp = connect(tcp_socket_, (struct sockaddr *)&addr_, sizeof(addr_));
    if (res_tcp == -1)
    {
      ROS_WARN("Connection failed, restarted in 5 seconds!");
      return -1;
    }
    ROS_INFO("Tcp Connection successful!");
    tcp_connected_flag_ = true;
    // follow_control_flag = false;
    // panel_control_flag = true;
    tcp_socket_used_flag = true;
    return 0;
  }

  //map回传线程，将本地地图发回到指挥端
  void Transmitter::StartFileTcpThread()
  {
    file_tcp_keep_running_ = true;
    file_tcp_thread_ = std::thread(
    std::bind(&Transmitter::FileTcpLoop, this));
  }

  // TCP线程，循环判断tcp是否连接
  void Transmitter::FileTcpLoop()
  {
    ROS_INFO("File Tcp thread is started!");
    FileTcp_setup();
    FileTcp_connect();
    int send_len;
    char buffer[4096];
    FILE *fq;

    if ((fq = fopen(FILEPATH, "rb")) == NULL)
    {
      ROS_INFO("File open failed!");
    }
    else
    {
      write(file_tcp_socket_, "begin", sizeof("begin"));
      bzero(buffer, sizeof(buffer));
      int count = 0;
      while (!feof(fq))
      {
        send_len = fread(buffer, 1, sizeof(buffer), fq);
        count++;
        if (send_len != write(file_tcp_socket_, buffer, send_len))
        {
          break;
        }
      }
      write(file_tcp_socket_, "end", sizeof("end"));
      fclose(fq);
    }
    ROS_INFO("File transfer done!");
    close(file_tcp_socket_);
  }

  //初始化tcp和udp的scoket，其中tcp连接用于和指挥端通信，udp连接用于向运动主机发送控制信息
  void Transmitter::FileTcp_setup()
  {
    file_tcp_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (file_tcp_socket_ == -1)
    {
      ROS_INFO("File Socket create failed!");
    }
    //std::cout << "Video Socket create" <<tcp_socket_<< std::endl;
    file_addr_.sin_family = AF_INET;
    file_addr_.sin_port = htons(FILEPORT);
    //addr_.sin_addr.s_addr = inet_addr(HOST_IP);
    file_addr_.sin_addr.s_addr = inet_addr(host_ip_);
  }

  //尝试tcp连接
  int Transmitter::FileTcp_connect()
  {
    int file_res_tcp = connect(file_tcp_socket_, (struct sockaddr *)&file_addr_, sizeof(file_addr_));
    if (file_res_tcp == -1)
    {
      ROS_INFO("Connection failed, please restart file thread!");
      return -1;
    }
    ROS_INFO("File Tcp Connection successful！");
    return 0;
  }

    //开启tcp线程
  void Transmitter::StartTcpThread()
  {
    tcp_keep_running_ = true;
    tcp_thread_ = std::thread(
        std::bind(&Transmitter::TcpLoop, this, tcp_thread_period_ms_));
    tcp_thread_started_ = true;
  }

  // TCP线程，判断tcp是否连接进行自动重连;接受指挥端信息进行处理
  void Transmitter::TcpLoop(int32_t period_ms)
  {
    ROS_INFO("Tcp thread is started!");
    Tcp_setup();
    while (tcp_keep_running_)
    {
      //自动重连，重新创建socket并发起连接
      while (!tcp_connected_flag_ && tcp_keep_running_)
      {
        if (tcp_socket_used_flag)
        {
          Tcp_setup();
        }
        else
        {
          Tcp_connect();
        }
        sleep(5);
      }
      // read the msg and parse it
      if (tcp_connected_flag_)
      {
        uint8_t rx_msg[BUFFER_SIZE]; // tcp read buffer
        //阻塞读取数据
        int rx_len = read(tcp_socket_, rx_msg, BUFFER_SIZE);
        //ROS_INFO("Receive msg from server,length is:%d", rx_len);

        //接收长度为0,代表当前tcp连接中断，继续尝试新的连接
        if (rx_len == 0)
        {
          ROS_INFO("Tcp disconnect, try to connect again!");
          tcp_connected_flag_ = false;
          mode  = -1;
          next_linear_speed = 0;
          next_angular_speed = 0;
          next_translation_speed = 0;
          close(tcp_socket_);
        }
        //接收到控制信息
        if (rx_len > 0)
        {
          //ROS_INFO_STREAM("Reveive control code:" << rx_msg);
          // 无人够状态控制指令
          if (rx_msg[0] == 'd')
          {
            uint32_t value_temp;
            //ROS_INFO_STREAM("Reveive control code:" << rx_msg);
            switch (rx_msg[1])
            {
            case '1': // 起立/趴下 指令码
              SendSimpleUDP(1, 0);
              ROS_INFO_STREAM("Reveive control msg: stand up/down.");
              break;
            case '2': // 力控 指令码
              SendSimpleUDP(2, 0);
              ROS_INFO_STREAM("Reveive control msg: force");
              break;
            case '3': // 踏步 指令码
              SendSimpleUDP(3, 0);
              ROS_INFO_STREAM("Reveive control msg: step.");
              break;
            case '5': // 翻倒(趴下时) 指令码
              SendSimpleUDP(5, 0);
              ROS_INFO_STREAM("Reveive control msg: turn down.");
              break;
            case '7': // 抬腿高度(轮换) 指令码
              SendSimpleUDP(7, 0);
              ROS_INFO_STREAM("Reveive control msg: up leg.");
              break;
            case 'e': //导航模式 指令码
              ROS_INFO_STREAM("Reveive control msg: navigation mode.");
              mode = 2;
              SendSimpleUDP(23, 0);
              // if(goal_status = 1){
              //   CancelNavi();
              // }
              if (serial_keep_running_)
              {
                // serial_keep_running_ = false;
                // usleep(20);
                pos_mutex.lock();
                current_x_position = 0;
                current_y_position = 0;
                next_x_position = 0;
                next_y_position = 0;
                pos_mutex.unlock();
              }
              break;
            case 'x': //xunluo模式 指令码
              ROS_INFO_STREAM("Reveive control msg: patrol mode.");
              mode = 3;
              continue_flag = true;
              circle_flag = false;
              nav_done = true;
              SendSimpleUDP(23, 0);
              // if(goal_status = 1){
              //   CancelNavi();
              // }
              if (serial_keep_running_)
              {
                // serial_keep_running_ = false;
                // usleep(20);
                pos_mutex.lock();
                current_x_position = 0;
                current_y_position = 0;
                next_x_position = dis_x;
                next_y_position = dis_y;
                pos_mutex.unlock();
              }
              break;
            case 'f': // 手动模式 指令码
              ROS_INFO_STREAM("Reveive control msg: hand control mode.");
              mode = 0;
              SendSimpleUDP(24, 0);
              if(goal_status = 1){
                CancelNavi();
              }
              cooperation_flag = false;
              if (serial_keep_running_)
              {
                // serial_keep_running_ = false;
                // usleep(20);
                pos_mutex.lock();
                current_x_position = 0;
                current_y_position = 0;
                next_x_position = dis_x;
                next_y_position = dis_y;
                pos_mutex.unlock();
              }
              break;
            case 's': // FOllow模式 指令码
              ROS_INFO_STREAM("Reveive control msg: follow mode.");
              SendSimpleUDP(23, 0);
              mode = 1;
              // if(goal_status = 1){
              //   CancelNavi();
              // }
              dis_x = dis_man;
              //StartSerialThread();
              break;
            case 'h': // 实时位置 指令码
              //SendSimpleUDP(52, 0);
              break;
            case 'g': // 切换步态 指令码
              SendSimpleUDP(25, 0);
              ROS_INFO_STREAM("Reveive control msg: switch gait.");
              break;
            case 'b': // 保存数据 指令码
              SendSimpleUDP(18, 0);
              break;
            case 'd'://心跳包
              SendSimpleUDP(21, 0);
              break;
            case '9': // 转向 指令码
              value_temp = (int)(rx_msg[3] - 48) * 10000 + (int)(rx_msg[4] - 48) * 1000 +
                           (int)(rx_msg[5] - 48) * 100 + (int)(rx_msg[6] - 48) * 10 + (int)(rx_msg[7]);
              if (rx_msg[2] == '0')
              {
                next_angular_speed = value_temp;
              }
              else if (rx_msg[2] == '-')
              {
                next_angular_speed = -value_temp;
              }
              //SendSimpleUDP(9, value_temp);
              ROS_INFO_STREAM("Reveive control msg: turning.");
              break;
            case 'a': //前进后退 指令码
              value_temp = (int)(rx_msg[3] - 48) * 10000 + (int)(rx_msg[4] - 48) * 1000 +
                           (int)(rx_msg[5] - 48) * 100 + (int)(rx_msg[6] - 48) * 10 + (int)(rx_msg[7]);
              if (rx_msg[2] == '0')
              {
                next_linear_speed = value_temp;
              }
              else if (rx_msg[2] == '-')
              {
                next_linear_speed = -value_temp;
              }
              //SendSimpleUDP(10, value_temp);
              ROS_INFO_STREAM("Reveive control msg: move.");
              break;
            case 'i': //平移 指令码
              value_temp = (int)(rx_msg[3] - 48) * 10000 + (int)(rx_msg[4] - 48) * 1000 +
                           (int)(rx_msg[5] - 48) * 100 + (int)(rx_msg[6] - 48) * 10 + (int)(rx_msg[7]);
              if (rx_msg[2] == '0')
              {
                next_translation_speed = value_temp;
              }
              else if (rx_msg[2] == '-')
              {
                next_translation_speed = -value_temp;
              }
              ROS_INFO_STREAM("Reveive control msg: translation.");
              break;
            case 'z': //emergency stop 指令码
              SendSimpleUDP(0x21010C0E, 0);
              ROS_INFO_STREAM("Reveive control msg: emergency stop.");
              break;
            default:
              break;
            }
          }
          //reveive navigation goal
          else if (rx_msg[0] == 'p')
          {
            if(rx_msg[1] == 'x')
            {
              CancelNavi(); 
              ROS_INFO_STREAM("Reveive control msg: cancel navigation goal.");
            }
            else
            {
            //图片坐标系转换到map坐标系
            int pix_x_tem = (int)(rx_msg[1] - 48) * 1000 + (int)(rx_msg[2] - 48) * 100 +
                            (int)(rx_msg[3] - 48) * 10 + (int)(rx_msg[4] - 48);
            int pix_y_tem = (int)(rx_msg[6] - 48) * 1000 + (int)(rx_msg[7] - 48) * 100 +
                            (int)(rx_msg[8] - 48) * 10 + (int)(rx_msg[9] - 48);
            double nav_x = pix_x_tem * resolution + origin_x;
            double nav_y = pix_y_tem * resolution + origin_y;

            //ROS_INFO_STREAM("nav goal pix (x,y:" << pix_x_tem << ", " << pix_y_tem);
            ROS_INFO_STREAM("Reveive navigation goal (x,y:" << nav_x << ", " << nav_y);
            Publish_Goal(nav_x, nav_y);
            }
          }
          //接收巡逻模式下的巡逻点坐标，存放到巡逻目标点列表中
          else if (rx_msg[0] == 'q')
          {
            if (rx_msg[1] == 'x')
            {
              if (continue_flag)
              {
                CancelNavi();
                nav_done = true;
              }
              
              // continue_flag = !continue_flag;
            }
            else
            {
              nav_done = false;
              first_nav = true;
              goal_count = 0;
              if (rx_msg[1] == '0')
              {
                circle_flag = false;
              }
              else if (rx_msg[1] == '1')
              {
                circle_flag = true;
              }
              ROS_INFO_STREAM(" circle_flag:" << circle_flag);
              goal_vec.clear();
              geometry_msgs::PointStamped p;
              int goal_len = (int)(rx_msg[2] - 48) * 10 + (int)(rx_msg[3] - 48);
              //读取预订长度的巡逻点，push到向量中保存
              for (int ii = 0; ii < goal_len; ii++)
              {
                int len_pos = 8 * ii + 4;
                int goal_x_tem = (int)(rx_msg[len_pos] - 48) * 1000 + (int)(rx_msg[len_pos + 1] - 48) * 100 +
                                 (int)(rx_msg[len_pos + 2] - 48) * 10 + (int)(rx_msg[len_pos + 3] - 48);
                int goal_y_tem = (int)(rx_msg[len_pos + 4] - 48) * 1000 + (int)(rx_msg[len_pos + 5] - 48) * 100 +
                                 (int)(rx_msg[len_pos + 6] - 48) * 10 + (int)(rx_msg[len_pos + 7] - 48);
                p.point.x = goal_x_tem * resolution + origin_x;
                p.point.y = goal_y_tem * resolution + origin_y;

                goal_vec.push_back(p);
                ROS_INFO_STREAM("goal_vec " << ii << ":" << goal_vec[ii].point.x << ", " << goal_vec[ii].point.y);
                //ROS_INFO_STREAM(" goal_tem" << ii << ":" << goal_x_tem << ", " << goal_y_tem);
              }
            }
            // for (int ii = 0; ii < goal_vec.size(); ii++)
            // {
            //   ROS_INFO_STREAM("goal_vec "<< ii << ":" << goal_vec[ii]);
            // }
          }
          //协同跟随模式
          else if (rx_msg[0] == 'x')
          {            
            //double x, y;
            ROS_INFO_STREAM("Reveive control code:" << rx_msg);
            //初始化命令，广播信息
            if(rx_msg[1] == 'i')
            {
              receive_leader = false;
              int jj = 2;
              while(jj < sizeof(rx_msg))
              {
                //判断队伍类型
                if(rx_msg[jj] == 't'){
                  my_team = rx_msg[jj+1];
                  jj = jj + 2;
                  continue;
                }
                //领队者编号以及位置
                else if(rx_msg[jj] == 'f'){
                  //自身为领队者
                   if(my_id == (int)(rx_msg[jj+1])){
                      cooperation_flag = true;
                      break;
                   } 
                   //保存领队者位置用于编队
                   else{
                     int leader_x = (int)(rx_msg[jj+2] - 48) * 1000 + (int)(rx_msg[jj+3] - 48) * 100 +
                            (int)(rx_msg[jj+4] - 48) * 10 + (int)(rx_msg[jj+5] - 48);
                      int leader_y = (int)(rx_msg[jj+6] - 48) * 1000 + (int)(rx_msg[jj+7] - 48) * 100 +
                                      (int)(rx_msg[jj+8] - 48) * 10 + (int)(rx_msg[jj+9] - 48);
                      int leader_angle = (int)(rx_msg[jj+11] - 48) * 100 +
                                      (int)(rx_msg[jj+12] - 48) * 10 + (int)(rx_msg[jj+13] - 48);
                      if(rx_msg[jj+10] == '0'){
                       leader_pos.point.z = leader_angle / 100;
                      }
                      else if(rx_msg[jj+10] == '-'){
                        leader_pos.point.z = -(double)leader_angle / 100.0;
                      }
                      leader_pos.point.x = leader_x * resolution + origin_x;
                      leader_pos.point.y = leader_y * resolution + origin_y;
                      //ROS_INFO_STREAM("leader pos0（x, y ,angle）"<<leader_x<<","<<leader_y<<", "<<leader_angle);
                      ROS_INFO_STREAM("leader pos(x,y,angle)"<<leader_pos.point.x<<","<<leader_pos.point.y<<","<<leader_pos.point.z);
                   }
                   jj += 14;
                   continue;
                }
                //跟随着编号以及编队位置
                else if(rx_msg[jj] == 'o'){
                  int k = jj+1;
                  while(rx_msg[k]!= 'e'){
                    if(my_id == int(rx_msg[k] - 48))
                    {
                      mode = 5;  
                      in_team = false;
                      first_go = true;
                      cooperation_flag = true;
                      //serial_keep_running_ = false;
                      //usleep(20);
                      pos_mutex.lock();
                      current_x_position = 0;
                      current_y_position = 0;
                      next_x_position = dis_x;
                      next_y_position = dis_y;
                      pos_mutex.unlock(); 
                      my_team_id = (int)(rx_msg[k+1] - 48);  
                      break;            
                    }
                    k += 2;
                  } 
                  break;
                }
              }
              ROS_INFO_STREAM("mode: "<<mode<<" my_team: "<<my_team<<" my_team_id: "<<my_team_id);
            }
            else if(rx_msg[1] == 'p')
            {
              int index = rx_len - 10;
              receive_leader = true;
              point_count = 0;
              int f1_x = (int)(rx_msg[index + 2] - 48) * 1000 + (int)(rx_msg[index + 3] - 48) * 100 +
                            (int)(rx_msg[index + 4] - 48) * 10 + (int)(rx_msg[index + 5] - 48);
              int f1_y = (int)(rx_msg[index + 6] - 48) * 1000 + (int)(rx_msg[index + 7] - 48) * 100 +
                                      (int)(rx_msg[index + 8] - 48) * 10 + (int)(rx_msg[index + 9] - 48);
              double f_x = f1_x * resolution + origin_x;
              double f_y = f1_y * resolution + origin_y; 
              Point_Map2Base(f_x, f_y);
              //Kalman(f_x, f_y); //                  
            }
          }
          //sendto(udp_socket_,rx_msg,sizeof(rx_msg),0,(struct sockaddr*)&udp_addr_,sizeof(udp_addr_));
          // ParseTcpBuffer(rx_msg, rx_len);
          memset(rx_msg, '\0', sizeof(rx_msg));
          rx_len = 0;
        }
      }
      // usleep(period_ms * 1000);
    }
    close(tcp_socket_);
    ROS_INFO("Tcp loop terminate!");
  }

    //开启控制线程，手动模式下udp发送速度;跟随模式下跟随AOA信标;巡逻模式下循环发布已保存的目标点
  void Transmitter::StartCmdThread()
  {
    cmd_keep_running_ = true;
    cmd_thread_ = std::thread(
        std::bind(&Transmitter::CmdLoop, this, cmd_thread_period_ms_));
    cmd_thread_started_ = true;
  }

  //控制线程:，手动模式下udp发送速度;
  //跟随模式下跟随AOA信标;
  //巡逻模式下循环发布已保存的目标点
  void Transmitter::CmdLoop(int32_t period_ms)
  {
    ROS_INFO("Control thread is started!");

    // int goal_count;
    int f_count = 0;
    int collision_dis = dis_x;
    while (cmd_keep_running_)
    {
      SendSimpleUDP(21, 0); //持续发送心跳包
      //手动控制模式，速度已在tcp接收时保存。直接发送
      if (mode == 0)
      {
        //motion_cmd_pub_.publish(next_twist_cmd_);
        SendSpeed();
      }

      //跟随模式，更新跟随目标位置，follow函数计算跟随速度，然后udp发送
      //(可以考虑cmd_vel方式，无人狗自带udp转发的节点)
      if (mode == 1)
      {
        //aoa标签打开时进行跟随控制
        if (tag_open_flag)
        {
          // f_count++;
          // if(f_count > 25){
          //   f_count = 0;
          //   FollowControl();
          // }          
          //Point_Base2Map)
          FollowControl();
        }
        //aoa标签关闭时速度为0
        else
        {
          next_x_position = dis_x;
          next_y_position = dis_y;
          next_twist_cmd_.linear.x = 0;
          next_twist_cmd_.angular.z = 0;
        }
        //ROS_INFO_STREAM("tag open flag: " << tag_open_flag);
        //SendSpeed(); //send current speed to motion cpmputer
       // motion_cmd_pub_.publish(next_twist_cmd_);
      }
      //巡逻模式:不循环时从初始点一圈回到初始点，循环时一直发布直到终止
      if (mode == 3) 
      {
        //ROS_INFO_STREAM("continue_flag "<< continue_flag<<" first_nav "<<first_nav<<" nav_done" <<nav_done); 
        if (!goal_vec.empty() && continue_flag)
        {
          //第一次进行导航，防止没有goal_status出错
          if (goal_status == -1)
          {
            Publish_Goal(goal_vec[0].point.x, goal_vec[0].point.y);
            goal_count = 1;
            ROS_INFO_STREAM("N av_goal" << goal_count << ":" << goal_vec[goal_count].point.x << ", " << goal_vec[goal_count].point.y);
            //ROS_INFO_STREAM("goal_count:" << goal_count);
          }
          //正在前往目标点，下次goal_status为3时是下一导航点
          else if (goal_status == 1)
          {
            first_nav = true;
          }
          else if (goal_status == 2)
          {
            first_nav = true;
          }
          else if (goal_status == 4)
          {
            first_nav = true;
          }
          //当前导航点已到达，发布下一导航点
          else if (goal_status == 3)
          {
            //ROS_INFO_STREAM("first flag"<<first_nav);
            if (first_nav)
            {
              if (!nav_done)
              {
                ROS_INFO_STREAM("count" << goal_count);
                if(goal_count == goal_vec.size())
                {
                  if (circle_flag)
                  {
                    goal_count = 0;
                    Publish_Goal(goal_vec[goal_count].point.x, goal_vec[goal_count].point.y);
                    ROS_INFO_STREAM("Nav_goal" << goal_count << ":" << goal_vec[goal_count].point.x << ", " << goal_vec[goal_count].point.y);
                    first_nav = false;
                  }
                  else
                  {
                    goal_count = 0;
                    Publish_Goal(goal_vec[goal_count].point.x, goal_vec[goal_count].point.y);
                    ROS_INFO_STREAM("Nav_goal" << goal_count << "" << goal_vec[goal_count].point.x << ", " << goal_vec[goal_count].point.y);
                    first_nav = false;
                    nav_done = true;
                  }                 
                }
                else
                {
                  Publish_Goal(goal_vec[goal_count].point.x, goal_vec[goal_count].point.y);
                  ROS_INFO_STREAM("Nav_goal： " << goal_count << ":" << goal_vec[goal_count].point.x << ", " << goal_vec[goal_count].point.y);
                  first_nav = false;
                }  
                goal_count++;              
                // if (goal_count < goal_vec.size())
                // {
                //   goal_count++;
                // }
              }
            }
          }
          //The goal can't reach
          else if (goal_status == 5)
          {
            if (!first_nav)
            {
              first_nav = true;
              if (goal_count < goal_vec.size() - 1)
              {
                goal_count++;
              }
            }
          }
        }
      }

      //co mode
      if(mode == 5)
      {
        if(receive_leader){
          //follow team member
          //if(receive_leader){
            if(point_count <= 10){
              point_count++;
              //collision avoid 
              //collision_dis = next_x_position - current_x_position;
              // if(next_x_position <= 120 && collision_dis <= -10){
              //   SendSimpleUDP(0x21010C0E, 0);
              //   ROS_INFO("Too close, Emergency stop!");
              // }
              FollowControl();
              motion_cmd_pub_.publish(next_twist_cmd_);
              // f_count++;
              // if(f_count > 25){
              //   f_count = 0;
              //   Point_Base2Map(next_x_position, next_y_position);
              // }
            }
            else{
                      pos_mutex.lock();
                      current_x_position = 0;
                      current_y_position = 0;
                      next_x_position = dis_x;
                      next_y_position = dis_y;
                      pos_mutex.unlock(); 
            }
          //}
        }
        else{
          if(first_go){
            CoTeam(&leader_pos, my_team, my_team_id);
            usleep(50000);
          }
          if(goal_status == 1){
            first_go = false;
          }
          else if(goal_status == 3 && !first_go){
             write(tcp_socket_, "xfinish", sizeof("xfinish"));
             ROS_INFO("Send team done!");
             in_team = true;
             usleep(50000);
          }
        }
      }

      //SendSpeed(); //send current speed to motion cpmputer
      usleep(period_ms * 1000);
    }
    ROS_INFO("Control loop terminate!");
  }

  //以无人狗协议UDP发送当前的速度到运动主机
  void Transmitter::SendSpeed()
  {
    bool linear_flag = false;
    bool angular_flag = false;
    bool translation_flag = false;
    if (next_angular_speed != 0)
    {
      angular_flag = true;
    }
    else if (current_angular_speed == 0)
    {
      angular_flag = false;
    }
    //速度为0的时候不发送指令
    if (angular_flag)
    {
      current_angular_speed = next_angular_speed;
      SendSimpleUDP(9, (uint32_t)current_angular_speed);
    }
    if (next_linear_speed != 0)
    {
      linear_flag = true;
    }
    else if (current_linear_speed == 0)
    {
      linear_flag = false;
    }
    //速度为0的时候不发送指令
    if (linear_flag)
    {
      current_linear_speed = next_linear_speed;
      SendSimpleUDP(10, (uint32_t)current_linear_speed);
    }

    if (next_translation_speed != 0)
    {
      translation_flag = true;
    }
    else if (next_translation_speed == 0 && current_translation_speed == 0)
    {
      translation_flag = false;
    }
    if (translation_flag)
    {
      current_translation_speed = next_translation_speed;
      SendSimpleUDP(110, (uint32_t)current_translation_speed);
    }
  }

  //跟随当前的目标位置
  void Transmitter::FollowControl()
  {
    //第一次进入循环时初始化当前位置
          if (current_x_position == 0)
          {
            current_x_position = next_x_position;
          }
          if (current_y_position == 0)
          {
            current_y_position = next_x_position;
          }
          //位置变动在5-50之间为有效数据
          int delta_x_pos = abs(next_x_position - current_x_position);
          int delta_y_pos = abs(next_y_position - current_x_position);

          if (delta_x_pos > 5 || delta_x_pos < 50)
          {
            current_x_position = next_x_position;
          }
          if (delta_y_pos > 5 || delta_y_pos < 50)
          {
            current_y_position = next_y_position;
          }

          //判断当前位置与设定跟踪位置差异，一段时间内都保持在一定差异内认为车辆可以进入静止状态
          int delta_follow_x = abs(current_x_position - dis_x);
          int delta_follow_y = abs(current_y_position - dis_y);

          if (static_flag)
          {
            //std::cout << "static state!" << std::endl;
            next_twist_cmd_.linear.x = 0;
            next_twist_cmd_.angular.z = 0;
            if (delta_follow_y > 20 || delta_follow_x > 20)
            {
              if (static_count >= 5)
              {
                static_count = 0;
                static_flag = false;
              }
              else
              {
                static_count++;
              }
            }
          }
          else
          {
            //小车为跟随状态，跟随当前tag位置
            follow(current_x_position, current_y_position);
            //Point_Base2Map(current_x_position, current_y_position);
            //角度跟随接近时间隔发送角速度，更为稳定
            if (angular_close)
            {
              t_count++;
              if (t_count <= 5)
              {
                next_twist_cmd_.angular.z = 0;
              }
              else
              {
                t_count = 0;
              }
            }
            else
            {
              t_count = 0;
            }
            //长时间在一定位置内判断进入静止模式
            if (delta_follow_y <= 20 && delta_follow_x <= 20)
            {
              if (static_count >= 50)
              {
                static_count = 0;
                static_flag = true;
              }
              else
              {
                static_count++;
              }
            }
          }
  }

    //无人狗状态返回线程
  void Transmitter::StartBackThread()
  {
    back_keep_running_ = true;
    back_thread_ = std::thread(
        std::bind(&Transmitter::BackLoop, this, back_thread_period_ms_));
    back_thread_started_ = true;
  }

  // 发送无人狗状态信息至指挥端
  void Transmitter::BackLoop(int32_t period_ms)
  {
    ROS_INFO("Data back thread is started!");
    while (back_keep_running_)
    {
      //robot state send back
      if (tcp_connected_flag_)
      {
        std::string voltage_str = std::to_string(battery);
        std::string gaint_str = std::to_string(robot_gait);
        std::string base_str = std::to_string(robot_base);
        int mode_tem;
        switch(mode){
          case 0:
            mode_tem = 0;//hand control ing
            break;
          case 1:
            mode_tem = 1;//follow -ing
            break;
          case 2:
            if(goal_status == 1){
              mode_tem = 2;//nav  -ing 
            }
            else{
              mode_tem = 3;//nav done
            }
            break;
          case 3:
            if(nav_done){
              mode_tem = 5;//xunluo done
            }
            else{
              mode_tem = 4;//xunluo -ing
            }
            break;
          default:
            mode_tem = 0;
            break;
        }
        //ROS_INFO_STREAM(mode<< "," <<mode_tem);
        std::string control_str = std::to_string(mode_tem);
        std::string x_str = std::to_string(pix_x);
        std::string y_str = std::to_string(pix_y);
        std::string w_str = std::to_string(pos_angle);
        std::string tx_str = voltage_str + ',' + gaint_str + ',' + base_str + ',' + control_str + ',' +
                             x_str + ',' + y_str + ',' + w_str + '\0';
        int num = sizeof(tx_str);
        char tx_msg2[64];

        strcpy(tx_msg2, tx_str.c_str());
        write(tcp_socket_, tx_msg2, 64);
        usleep(period_ms * 1000);
        //std::cout << "tx msg:" << tx_msg2 <<","<<sizeof(tx_msg2)<< std::endl;
        // std::cout << "tx_str: " << tx_str << ",size " << num << std::endl;
      }
      //std::cout << "heart!" << std::endl;
      //usleep(period_ms * 1000);
    }
    ROS_INFO("Data back loop terminate!");
  }

    //开启串口循环线程
  void Transmitter::StartSerialThread()
  {
    serial_keep_running_ = true;
    serial_thread_ = std::thread(
        std::bind(&Transmitter::SerialLoop, this, serial_thread_period_ms_));
    serial_thread_started_ = true;
  }

  //循环读取串口的数据，根据aoa基站协议解析
  void Transmitter::SerialLoop(int32_t period_ms)
  {
    ROS_INFO("Serial thread is started!");
    while (serial_keep_running_)
    {
      while (!serial_.isOpen() & serial_keep_running_)
      {
        Open_Serial(device_name, baud_rate);
        if (!serial_.isOpen())
        {
          sleep(5);
        }
        if (serial_.isOpen())
        {
          serial_.flushInput();
        }
      }
      if (serial_.isOpen())
      {
        size_t length = serial_.available();
        //int tag_open_count = 0;
        if (length != 0)
        {
          tag_open_flag = true;
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
        }
        //10次为收到tag信息认为tag处于关闭状态
        else if (length == 0)
        {
          //ROS_INFO("Tag close!");
          tag_open_count++;
          if (tag_open_count >= 10)
          {
            //ROS_INFO("Tag close 10!");
            tag_open_count = 0;
            tag_open_flag = false;
            pos_mutex.lock();
            next_x_position = dis_x;
            next_y_position = dis_y;
            pos_mutex.unlock();
          }
        }
      }
      usleep(period_ms * 1000);
    }
    ROS_INFO("Serial loop terminate!");
  }

  //根据AOA基站的数据协议，解析当前tag的X和Y坐标
  void Transmitter::ParseUARTBuffer(uint8_t *buf, int bytes_received)
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
        //position_mutex_.unlock();
        //   std::cout << "Ycm" << y_pos <<std::endl;
        // std::cout << "Ycm" << recv_y <<std::endl;
      }
    }
    //对收到的坐标进行卡尔曼滤波
    if(mode == 1){
      Kalman(recv_y, recv_x);
    }   
  }

  //无人狗简单指令的协议，指令码+指令值
  void Transmitter::SendSimpleUDP(int code_, uint32_t size)
  {
    struct CommandHead command_head = {0};
    command_head.code = code_;
    command_head.paramters_size = size;
    command_head.type = 0;
    sendto(udp_socket_, (uint8_t *)&command_head, sizeof(command_head), 0, (sockaddr *)&udp_addr_, sizeof(udp_addr_));
  }

  //complex udp command,contain code and code value
  void Transmitter::SendComplexUDP(int code_, int32_t value_)
  {
    struct Command command = {0};
    command.head.code = code_;
    int32_t to_send_data[64];
    to_send_data[0] = 32000;

    command.data = value_;
    //memcpy(&command.data,&to_send_data,sizeof(to_send_data));
    command.head.paramters_size = 0;
    command.head.type = 0;

    std::cout << "to_send_data" << &to_send_data << std::endl;
    std::cout << "command.data" << &command.data << std::endl;
    std::cout << "paramters_size" << command.head.paramters_size << std::endl;
    sendto(udp_socket_, (uint8_t *)&command, sizeof(command), 0, (sockaddr *)&udp_addr_, sizeof(udp_addr_));
  }

  //odom话题回调函数，无人狗上odom即为在map中的坐标
  //转换为地图像素坐标和yaw角后保存
  void Transmitter::OdomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
  {
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    pos_angle = yaw;

    pix_x = (pos_x - origin_x) / resolution;
    pix_y = (pos_y - origin_y) / resolution;
    //std::cout<<"pos_x"<<pos_x<<", pos_y"<<pos_y<<", pix_x"<<pix_x<<", pix_y"<<pix_y<<std::endl;
  }

   //无人狗电量百分比
  void Transmitter::BatteryCallBack(const std_msgs::Float32::ConstPtr &msg)
  {
    battery = msg->data;
    //std::cout << "battery:" << battery << std::endl;
  }

 void Transmitter::MotortemCallBack(const std_msgs::Float32::ConstPtr &msg)
  {
  }

    void Transmitter::DrivertemCallBack(const std_msgs::Float32::ConstPtr &msg)
  {
  }

   //无人狗当前状态(趴下，站立，力控，踏步站立等)
  void Transmitter::BasicStateCallBack(const std_msgs::Int32::ConstPtr &msg)
  {
    robot_base = msg->data;
    //std::cout << "robot_base:" << robot_base << std::endl;
  }

    //无人狗步态
  void Transmitter::GaitStateCallBack(const std_msgs::Int32::ConstPtr &msg)
  {
    robot_gait = msg->data;
    //std::cout << "robot_gait:" << robot_gait << std::endl;
  }

    //无人狗控制模式(手动，导航)
  void Transmitter::ControlModeCallBack(const std_msgs::Int32::ConstPtr &msg)
  {
    control_mode = msg->data;
    //std::cout << "control_mode:" << control_mode << std::endl;
  }

    //导航点状态 1正在前往 3已到达
  void Transmitter::GoalStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses)
  {
    if (!statuses->status_list.empty())
    {
      for (auto &i : statuses->status_list)
      {
        cur_goalid_ = i.goal_id;
        goal_status = i.status;
      }
    }
  }

  //将map中的一个点坐标转换到baselink坐标系
  void Transmitter::Point_Map2Base(double x_pos, double y_pos)
  {
    geometry_msgs::PointStamped map_pos;  //创建map一个点
    geometry_msgs::PointStamped base_pos; //创建baselink一个点
    map_pos.header.frame_id = "map";      //将这个点绑定到雷达坐标系下
    map_pos.header.stamp = ros::Time();
    map_pos.point.x = x_pos;
    map_pos.point.y = y_pos;
    map_pos.point.z = 0;

    // tf::TransformListener listener(ros::Duration(10))//等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
    listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(3)); //ros::Time(0)表示使用缓冲中最新的tf数据
    listener.transformPoint("base_link", map_pos, base_pos);                       //将map中的点变换到base_link中去
    ROS_INFO("map: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
             map_pos.point.x, map_pos.point.y, map_pos.point.z,
             base_pos.point.x, base_pos.point.y, base_pos.point.z, base_pos.header.stamp.toSec());

    double next_x_tem = base_pos.point.x * 100;
    double next_y_tem = base_pos.point.y * 100;
    if(next_x_tem <= 1500 && next_y_tem <= 1500){
      Kalman(next_x_tem, next_y_tem);
    }    
  }

//将map中的一个点坐标转换到baselink坐标系
  void Transmitter::Point_Base2Map(int x_pos1, int y_pos1)
  {
    double x_pos = (double)x_pos1 / 100;
    double y_pos = (double)y_pos1 / 100;
    geometry_msgs::PointStamped map_pos;    //创建map一个点
    geometry_msgs::PointStamped base_pos;   //创建baselink一个点
    base_pos.header.frame_id = "base_link"; //将这个点绑定到雷达坐标系下
    base_pos.header.stamp = ros::Time();
    // base_pos.point.x = x_pos;
    // base_pos.point.y = y_pos;
    base_pos.point.z = 0;

    double distance = sqrt(pow(x_pos, 2) +
                           pow(y_pos, 2));
    //double nav_angle = acos(y_pos / distance);
    // if (y_diff < 0)
    // {
    //   nav_angle = -nav_angle;
    // }
    ROS_INFO("follow pos: %d, %d", x_pos1, y_pos1);
    ROS_INFO("follow pos: %f, %f", x_pos, y_pos);
    x_pos = x_pos - 1 * x_pos / distance;
    y_pos = y_pos - 1 * y_pos / distance;
    
    base_pos.point.x = x_pos;
    base_pos.point.y = y_pos;
    // tf::TransformListener listener(ros::Duration(10))//等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
    listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3)); //ros::Time(0)表示使用缓冲中最新的tf数据
    listener.transformPoint("map", base_pos, map_pos);                             //将map中的点变换到base_link中去
    ROS_INFO("map: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
             base_pos.point.x, base_pos.point.y, base_pos.point.z,
             map_pos.point.x, map_pos.point.y, map_pos.point.z,
             base_pos.header.stamp.toSec());
    Publish_Goal(map_pos.point.x, map_pos.point.y);
  }

    //发布当前点为导航点，方向设定为本机指向目标点的角度
  void Transmitter::Publish_Goal(double nav_x, double nav_y)
  {
    geometry_msgs::PoseStamped goal;
    double x_diff = nav_x - pos_x;
    double y_diff = nav_y - pos_y;
    double distance = sqrt(pow(x_diff, 2) +
                           pow(y_diff, 2));
    double nav_angle = acos(x_diff / distance);
    if (y_diff < 0)
    {
      nav_angle = -nav_angle;
    }
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(nav_angle);
    // std::cout << "nav_angle:" << nav_angle << std::endl;
    goal.header.frame_id = "map";
    goal.pose.position.x = nav_x;
    goal.pose.position.y = nav_y;
    goal.pose.orientation = quat;
    goal_pub_.publish(goal);
    ROS_INFO_STREAM("Publish goal: " <<nav_x << ", "<< nav_y);
  }

//发布当前点为导航点，方向设定为本机指向目标点的角度
  void Transmitter::Publish_Goal_Co(double nav_x, double nav_y, double nav_angle)
  {
    geometry_msgs::PoseStamped goal;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(nav_angle);
    // std::cout << "nav_angle:" << nav_angle << std::endl;
    goal.header.frame_id = "map";
    goal.pose.position.x = nav_x;
    goal.pose.position.y = nav_y;
    goal.pose.orientation = quat;
    goal_pub_.publish(goal);
  }

   //取消当前导航点
  void Transmitter::CancelNavi()
  {
    // goal_count = 0;
    if (!cur_goalid_.id.empty())
    {
      cancel_pub_.publish(cur_goalid_);
      ROS_INFO_STREAM("Current goal id" << cur_goalid_);
      ROS_INFO_STREAM("Navigation have been canceled");
    }
  }

   //参数含义:Kalman_KX 卡尔曼系数  Kalman_XD 当前估计值 Kalman_ZX  上一次测量值
  //Kalman_ESTX 估计误差   Kalman_MEAX 测量误差
  void Transmitter::Kalman(double x_position, double y_position)
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
    pos_mutex.lock();
    next_x_position = Kalman_XA;
    next_y_position = Kalman_XD;
    pos_mutex.unlock();
    // std::cout<<"before kalman position"<<x_position<<","<<y_position<<std::endl;
    // std::cout<<"Kalman position"<<next_x_position<<","<<next_y_position<<std::endl;
  }

  // 根据当前跟随目标相对与本机的x，y位置，对比设定的跟随距离以及最大跟随距离，
  // 计算相应的线速度和角速度；在转弯时降低线速度
  void Transmitter::follow(int x_position, int y_position)
  {
    int delta_x, delta_y;
     double y_zoom;
    if (x_position >= 0)
    {
      delta_x = x_position - dis_x;

      if (delta_x >= -10 && delta_x <= 20)
      {
        next_twist_cmd_.linear.x = 0;
        y_zoom = 1;
      }
      else if(delta_x > 20 && delta_x < max_x)
      {
        next_twist_cmd_.linear.x = delta_x / max_x * max_linear_speed;
        y_zoom = 1 + delta_x / max_x;
      }
      else if(delta_x < -10 && delta_x >= -dis_x)
      {
        if(cooperation_flag)
        {
          next_twist_cmd_.linear.x = -0.2;
        }
        else{
          next_twist_cmd_.linear.x = -delta_x / dis_x;
        }
        y_zoom = 1;
      }
      // else if(delta_x < -dis_x/2 && delta_x >= -dis_x)
      // {
      //   next_twist_cmd_.linear.x = -0.3;
      //   y_zoom = 1;
      // }
      else if(delta_x >= max_x && delta_x <= 1500){
        next_twist_cmd_.linear.x = max_linear_speed;
        y_zoom = 2;
      }
      else if(delta_x > 1500){
        next_twist_cmd_.linear.x = 0;
        y_zoom = 1;
      }
      //ROS_INFO_STREAM("delta_x, " << delta_x << " next_linear_speed, " << next_twist_cmd_.linear.x);
    }

    double ratio_y;
    delta_y = -y_position - dis_y;
    ratio_y = delta_y / max_y;
    double ration_speed;
    int pos_delta_y = abs(delta_y);
    if (pos_delta_y <= 20)
    {
      ration_speed = 1;
      angular_close = true;
      next_twist_cmd_.angular.z = 0;
    }
    else if (pos_delta_y <= 30 * y_zoom)
    {
      ration_speed = 1;
      angular_close = true;
      next_twist_cmd_.angular.z = -0.15;
    }
    else if (pos_delta_y <= 60 * y_zoom)
    {
      ration_speed = 0.9;
      next_twist_cmd_.angular.z = -0.3;
      angular_close = false;
    }
    else if (pos_delta_y <= 100 *  y_zoom)
    {
      ration_speed = 0.8;
      next_twist_cmd_.angular.z = -0.45;
      angular_close = false;
    }
    else if (pos_delta_y > 100 * y_zoom)
    {
      ration_speed = 0.7;
      next_twist_cmd_.angular.z = -0.61;
      angular_close = false;
    }

    if (delta_y <= 0)
    {
      next_twist_cmd_.angular.z = -next_twist_cmd_.angular.z;
    }
    next_twist_cmd_.linear.x = next_twist_cmd_.linear.x * ration_speed;
    ROS_INFO_STREAM("Follow position (X,Y):" << x_position << ", " << y_position);
    ROS_INFO_STREAM("Follow speed (linear,angle):" << next_twist_cmd_.linear.x << ", " << next_twist_cmd_.angular.z);
  }

  //根据协同跟随初始化信息，计算本机处于队形中的位置，发布导航点
  void Transmitter::CoTeam(geometry_msgs::PointStamped *leader, uint8_t team_code, int team_id){
    geometry_msgs::PointStamped follower;
    if(team_code == '0'){
      dis_x = 200;
      dis_y = 0;
      follower.point.x = leader->point.x - 2 * (team_id - 1) * cos(leader->point.z);
      follower.point.y = leader->point.y - 2 * (team_id - 1) * sin(leader->point.z);
      follower.point.z = leader->point.z;
    }
    Publish_Goal_Co(follower.point.x, follower.point.y , follower.point.z);
  }
