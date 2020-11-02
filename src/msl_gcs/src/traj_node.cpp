#include <ros/ros.h>
#include <mavros_msgs/TrajectoryNominal.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<mavros_msgs::TrajectoryNominal>("chatter",1000);

    ros::Rate rate(20.0);

    while(ros::ok()){
        mavros_msgs::TrajectoryNominal msg;

        msg.timestamp = 1337.0f;

        for (int i = 0; i < 20; i++) {
            msg.f_out[i] = i;
        }

        traj_pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}