#include <ros/ros.h>
#include <mavros_msgs/TrajectoryNominal.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<mavros_msgs::TrajectoryNominal>("mavros/trajectory_nominal/trajectory_nominal_sub",1000);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        mavros_msgs::TrajectoryNominal msg;

        msg.timestamp = 1337;

        for (int i = 0; i < 20; i++) {
            msg.f_out[i] = 3.0f;
        }

        traj_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
