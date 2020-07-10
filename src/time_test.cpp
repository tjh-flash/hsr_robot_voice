#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <ros/duration.h>
#include <time.h>

std::string report_time(int a, int b);

//接收到订阅的消息后，会进入消息回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    //将接收到的信息答应出来
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    //获取当前时间
    struct tm *ptm; 
    long ts; 

    ts = time(NULL); 
    ptm = localtime(&ts);

    std::string time_result = report_time(ptm-> tm_hour, ptm-> tm_min);
    std::cout << time_result << std::endl;

}


int  main(int argc, char *argv[])
{
    //初始化ROS节点
    ros::init(argc, argv, "time_listener");
    
    //创建节点句柄
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("voiceSolve_res", 1000, chatterCallback);

    ros::spin();
    ros::waitForShutdown();

    return 0;
}

std::string report_time(int a, int b)
{

	int unitHour = a / 1 % 10;
	int tenHour = a / 10 % 10;
	int unitMinute = b / 1 % 10;
	int tenMinute = b / 10 % 10;

	
	std::string ten[6] = {"零", "十", "二十", "三十", "四十", "五十"};
	std::string unit_first[10] = {"零", "一", "二", "三", "四", "五", "六", "七", "八", "九"};
    std::string unit_end[10] = {"", "一", "二", "三", "四", "五", "六", "七", "八", "九"};
	std::string current_time;
	std::string day_period;

	if ( a <= 6)
		day_period = "凌晨";
	else if (a > 6 && a <= 11)
		day_period = "上午";
	else if (a > 11 && a <= 14)
		day_period = "中午";
	else if (a > 14 && a <= 18)
		day_period = "下午";
	else 
		day_period = "晚上";

	if (a < 10)
		current_time = "现在是北京时间" + day_period + unit_first[a] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else if (a = 10)
		current_time = "现在是北京时间" + day_period + ten[1] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else if (a >= 10 && a < 20)
	    current_time = "现在是北京时间" + day_period + ten[1] + unit_end[unitHour] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else if (a = 20)
		current_time = "现在是北京时间" + day_period + ten[2]  + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	else 
		current_time = "现在是北京时间" + day_period + ten[2] + unit_end[unitHour] + "点" + ten[tenMinute] + unit_end[unitMinute] + "分";
	
	return current_time;

}
	