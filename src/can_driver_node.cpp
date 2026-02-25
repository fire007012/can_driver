#include "can_driver/CanDriverHW.h"

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    CanDriverHW hw;
    if (!hw.init(nh, pnh)) {
        ROS_FATAL("[can_driver_node] Hardware init failed, exiting.");
        return 1;
    }

    controller_manager::ControllerManager cm(&hw, nh);

    // 2 个线程处理 service / topic 回调，主线程专用于控制循环
    ros::AsyncSpinner spinner(2);
    spinner.start();

    double freq = 100.0;
    pnh.param("control_frequency", freq, freq);
    ros::Rate rate(freq);

    ros::Time prev = ros::Time::now();
    while (ros::ok()) {
        ros::Time     now = ros::Time::now();
        ros::Duration dt  = now - prev;
        prev              = now;

        hw.read(now, dt);
        cm.update(now, dt);
        hw.write(now, dt);

        rate.sleep();
    }

    spinner.stop();
    return 0;
}
