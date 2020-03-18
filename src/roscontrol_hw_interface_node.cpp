#include <br_hw_interface/br_hw_interface.h>
#include <ros/callback_queue.h>

//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot

int main(int argc, char** argv)
{
    ros::init(argc, argv, "br_hardware_interface");

    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    br_hardware_interface::BRHardwareInterface rhi(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);

    return 0;
}