#include "flight_control.h"
#include "obstacle_avoidance.h"
#include "ring_detection.h"
#include "vision_detection.h"
#include "mission_callbacks.h"
#include "mission_header.h"
#include "servo.h"

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 全局回调变量（你工程里已有定义）
extern mavros_msgs::State current_state;
extern nav_msgs::Odometry local_pos;

int mission_num = 1;
float err_max = 0.2;
ros::Time last_request;
bool delay = false;

// 延时函数（用你原本的）
void Delay(float delay_time)
{
    if (delay)
    {
        ros::Duration delta_time = ros::Time::now() - last_request;
        if (ros::Time::now() - last_request >= ros::Duration(delay_time))
        {
            mission_num += 1;
            delay = false;
        }
    }
    else
    {
        last_request = ros::Time::now();
        delay = true;
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "vision_hand_hold_node");
    ros::NodeHandle nh;

    // 只订阅必要话题，不发布任何飞行控制、不解锁、不切Offboard
    ros::Subscriber state_sub       = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub    = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber image_sub        = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 10, image_cb);

    ros::Rate rate(20);

    ROS_INFO("======== 手持无人机视觉检测模式启动 ======== ");
    ROS_INFO("无需起飞、无需解锁、手持对着画面即可识别");

    while (ros::ok())
    {
        switch (mission_num)
        {
            // 第一步：先识别二维码
            case 1:
                if (detectQRCodeAndExtractInfo())
                {
                    ROS_INFO("二维码识别完成，进入物体分类检测");
                    Delay(1.0);
                }
                break;

            // 第二步：无限循环 黑框检测 + 图片分类
            case 2:
                // 你原有黑框+分类函数，一直循环检测
                onFrame(0, err_max);
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}