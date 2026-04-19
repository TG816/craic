#include "flight_control.h"
#include "obstacle_avoidance.h"
#include "ring_detection.h"
#include "vision_detection.h"
#include "mission_callbacks.h"
#include "mission_header.h"


// 空实现，防止报错
void print_param(){}
void Delay(float t){}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "flight_mission");
    ros::NodeHandle nh;

    // 订阅图像话题（你的视觉识别回调 image_cb 会自动跑）
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(
        "/camera/image_raw", 10, image_cb
    );

    ROS_INFO("========================================");
    ROS_INFO("      视觉识别测试节点 已启动 ✅");
    ROS_INFO("      只识别，不飞行，不控制无人机 ");
    ROS_INFO("========================================");

    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();  // 执行图像回调（识别）

        // ==============================================
        // 在这里加你要跑的 识别函数
        // 我给你写好示例，你直接替换成你的函数名即可
        // ==============================================
        
        // 示例1：环识别
        // ring_detection();
        
        // 示例2：视觉检测
        // vision_detection();
        
        // 示例3：你自己的识别函数
        // your_detection_function();

        // 你原来项目里的识别函数，直接放这里！
        // 比如：
        onFrame(0,0.1);
        // 或
        // detectThrowObject();


        rate.sleep();
    }

    return 0;
}