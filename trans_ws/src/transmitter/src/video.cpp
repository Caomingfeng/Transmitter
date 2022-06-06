
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
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv-3.3.1-dev/opencv/highgui.h>
#include <opencv-3.3.1-dev/opencv2/core.hpp>
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>

#define SERVERPORT 1234
#define host_ip_ "192.168.1.1"

using namespace std;
int video_tcp_socket_;
struct sockaddr_in addr_;

void VideoCallBack(const sensor_msgs::ImageConstPtr &msg)
{
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::vector<uchar> data_encode;
        std::vector<int> quality;
        quality.push_back(CV_IMWRITE_JPEG_QUALITY);
        quality.push_back(30);
        imencode(".jpg", cv_ptr->image, data_encode, quality); //将图像编码
        int nSize = data_encode.size();
        std::cout << nSize << std::endl;
        unsigned char *encodeImg = new unsigned char[nSize];
        for (int i = 0; i < nSize; i++)
        {
                encodeImg[i] = data_encode[i];
        }
        char size[5];
        sprintf(size, "%d", nSize);
        send(video_tcp_socket_,"be",3,0);
        send(video_tcp_socket_, size, 5, 0);
        send(video_tcp_socket_, encodeImg, nSize, 0);

        //cv::imshow("OPENCV_WINDOW", cv_ptr->image);
        //cv::waitKey(3);

        /* cap.read(frame);
            if(frame.empty()){
              std::cout << "读取图像错误！" << std::endl;
            }
            std::vector<uchar> data_encode;
            std::vector<int> quality;
            quality.push_back(CV_IMWRITE_JPEG_QUALITY);
            quality.push_back(30);//进行50%的压缩
            imencode(".jpg", frame, data_encode,quality);//将图像编码
            int nSize = data_encode.size();
            //std::cout << nSize << std::endl;
            unsigned char *encodeImg = new unsigned char[nSize];
            for (int i = 0; i < nSize; i++)
            {
              encodeImg[i] = data_encode[i];
            }
            char size[16];
            
            char be[3]= "be";
            

            sprintf(size,"%d",nSize);
            send(video_tcp_socket_,be,3,0);
            send(video_tcp_socket_,size,16,0);
            //std::cout << "be" << be<<std::endl;
            send(video_tcp_socket_,encodeImg,nSize,0);
            // count22++;
            // if(count22 == 2){
            //   video_tcp_keep_running_ = false;
            // }*/
}

int main(int argc, char **argv)
{
        //ROS initial
        //ROS initial
        ros::init(argc, argv, "video");
        ros::NodeHandle n;
        ros::Subscriber video_sub;
        //string host_ip_ = "192.168.1.3";

        video_tcp_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (video_tcp_socket_ == -1)
        {
                ROS_WARN("TCP Socket create failed!");
        }
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(SERVERPORT);
        //addr_.sin_addr.s_addr = inet_addr(HOST_IP);
        addr_.sin_addr.s_addr = inet_addr(host_ip_);

        int res_tcp = connect(video_tcp_socket_, (struct sockaddr *)&addr_, sizeof(addr_));
        if (res_tcp == -1)
        {
                ROS_WARN("Connection failed, restarted in 5 seconds!");
                exit(-1);
                //return -1;
        }
        ROS_INFO("Tcp Connection successful!");

        video_sub = n.subscribe(
            "/camera_front_up/color/image_raw", 1, &VideoCallBack);
        ros::spin();

        return 0;
}
