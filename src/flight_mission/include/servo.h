#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <string>
#include <vector>
#include <utility> 

#define PUT_ALTITUDE 0.20f

// 舵机偏移参数（修复：std:: + inline 防止重复定义）
inline std::vector<std::pair<float, float>> servo_offset = {
    {0.0, 0.0}, // 占位
    {0.0, 0.0}, // front_left  1
    {0.0, 0.0}, // front_right 2
    {0.0, 0.0}, // back_left   3
    {0.0, 0.0}  // back_right  4
};

class Servo
{
private:
    // 舵机相关发布器
    ros::Publisher servo_pub_all_open;
    ros::Publisher servo_pub_all_close;
    ros::Publisher servo_pub_front_left;
    ros::Publisher servo_pub_front_right;
    ros::Publisher servo_pub_back_left;
    ros::Publisher servo_pub_back_right;

    ros::Publisher servo_pub_front_left_open;
    ros::Publisher servo_pub_front_right_open;
    ros::Publisher servo_pub_back_left_open;
    ros::Publisher servo_pub_back_right_open;

    ros::Publisher servo_pub_front_left_close;
    ros::Publisher servo_pub_front_right_close;
    ros::Publisher servo_pub_back_left_close;
    ros::Publisher servo_pub_back_right_close;

public:
    /**
     * @brief 初始化所有舵机发布器
     * @param nh ROS节点句柄
     */
    void servo_init(ros::NodeHandle &nh)
    {
        // 全局开关
        servo_pub_all_open = nh.advertise<std_msgs::Empty>("/servo/all/open", 10);
        servo_pub_all_close = nh.advertise<std_msgs::Empty>("/servo/all/close", 10);

        // 角度控制发布器
        servo_pub_front_left = nh.advertise<std_msgs::Int32>("/servo/front_left", 10);
        servo_pub_front_right = nh.advertise<std_msgs::Int32>("/servo/front_right", 10);
        servo_pub_back_left = nh.advertise<std_msgs::Int32>("/servo/back_left", 10);
        servo_pub_back_right = nh.advertise<std_msgs::Int32>("/servo/back_right", 10);

        // 单个舵机 开/关 命令发布器
        servo_pub_front_left_open = nh.advertise<std_msgs::Empty>("/servo/front_left/open", 10);
        servo_pub_front_left_close = nh.advertise<std_msgs::Empty>("/servo/front_left/close", 10);
        servo_pub_front_right_open = nh.advertise<std_msgs::Empty>("/servo/front_right/open", 10);
        servo_pub_front_right_close = nh.advertise<std_msgs::Empty>("/servo/front_right/close", 10);

        servo_pub_back_left_open = nh.advertise<std_msgs::Empty>("/servo/back_left/open", 10);
        servo_pub_back_left_close = nh.advertise<std_msgs::Empty>("/servo/back_left/close", 10);
        servo_pub_back_right_open = nh.advertise<std_msgs::Empty>("/servo/back_right/open", 10);
        servo_pub_back_right_close = nh.advertise<std_msgs::Empty>("/servo/back_right/close", 10);
    }

    /**
     * @brief 控制所有舵机开关
     * @param mode open/close
     */
    void servo_all_control(std::string mode = "open")
    {
        std_msgs::Empty msg;
        ros::Publisher pub;
        if (mode == "open")
        {
            ROS_WARN("舵机全部打开！");
            pub = servo_pub_all_open;
        }
        else
        {
            ROS_WARN("舵机全部关闭！");
            pub = servo_pub_all_close;
        }
        pub.publish(msg);
    }

    /**
     * @brief 通过位置字符串控制单个舵机开关
     * @param pos 位置: front_left/front_right/back_left/back_right
     * @param mode open/close
     */
    void servo_control_better(std::string pos, std::string mode = "open")
    {
        ros::Publisher pub;
        std_msgs::Empty msg;
        if (pos == "front_left")
        {
            if (mode == "open")
            {
                pub = servo_pub_front_left_open;
                ROS_WARN("左前舵机打开！");
            }
            else
            {
                pub = servo_pub_front_left_close;
                ROS_WARN("左前舵机关闭！");
            }
        }
        else if (pos == "front_right")
        {
            if (mode == "open")
            {
                pub = servo_pub_front_right_open;
                ROS_WARN("右前舵机打开！");
            }
            else
            {
                pub = servo_pub_front_right_close;
                ROS_WARN("右前舵机关闭！");
            }
        }
        else if (pos == "back_left")
        {
            if (mode == "open")
            {
                pub = servo_pub_back_left_open;
                ROS_WARN("左后舵机打开！");
            }
            else
            {
                pub = servo_pub_back_left_close;
                ROS_WARN("左后舵机关闭！");
            }
        }
        else if (pos == "back_right")
        {
            if (mode == "open")
            {
                pub = servo_pub_back_right_open;
                ROS_WARN("右后舵机打开！");
            }
            else
            {
                pub = servo_pub_back_right_close;
                ROS_WARN("右后舵机关闭！");
            }
        }
        pub.publish(msg);
    }

    /**
     * @brief 通过数字编号控制舵机开关
     * @param num 0-全部 1-左前 2-右前 3-左后 4-右后
     * @param mode open/close
     */
    void servo_control_num_better(int num, std::string mode = "open")
    {
        switch (num)
        {
        case 0:
            servo_all_control(mode);
            break;
        case 1:
            servo_control_better("front_left", mode);
            break;
        case 2:
            servo_control_better("front_right", mode);
            break;
        case 3:
            servo_control_better("back_left", mode);
            break;
        case 4:
            servo_control_better("back_right", mode);
            break;
        default:
            ROS_ERROR("Wrong servo_control num!\n");
        }
    }

    /**
     * @brief 直接发送角度值控制舵机
     * @param pub 对应舵机的发布器
     * @param pos 舵机位置
     * @param mode open/close
     */
    void servo_control(ros::Publisher &pub, std::string pos, std::string mode = "open")
    {
        std_msgs::Int32 msg;
        if (pos == "front_left")
        {
            msg.data = (mode == "open") ? 180 : 30;
            ROS_WARN("左前舵机%s！", mode.c_str());
        }
        else if (pos == "front_right")
        {
            msg.data = (mode == "open") ? 180 : 0;
            ROS_WARN("右前舵机%s！", mode.c_str());
        }
        else if (pos == "back_left")
        {
            msg.data = (mode == "open") ? 180 : 0;
            ROS_WARN("左后舵机%s！", mode.c_str());
        }
        else if (pos == "back_right")
        {
            msg.data = (mode == "open") ? 180 : 30;
            ROS_WARN("右后舵机%s！", mode.c_str());
        }
        pub.publish(msg);
    }

    /**
     * @brief 通过数字编号发送角度控制舵机
     * @param num 0-全部 1-左前 2-右前 3-左后 4-右后
     * @param mode open/close
     */
    void servo_control_num(int num, std::string mode = "open")
    {
        switch (num)
        {
        case 0:
            servo_all_control(mode);
            break;
        case 1:
            servo_control(servo_pub_front_left, "front_left", mode);
            break;
        case 2:
            servo_control(servo_pub_front_right, "front_right", mode);
            break;
        case 3:
            servo_control(servo_pub_back_left, "back_left", mode);
            break;
        case 4:
            servo_control(servo_pub_back_right, "back_right", mode);
            break;
        default:
            ROS_ERROR("Wrong servo_control num!\n");
        }
    }
};

#endif // SERVO_CONTROL_H