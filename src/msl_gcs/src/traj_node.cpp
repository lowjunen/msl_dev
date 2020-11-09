#include <ros/ros.h>
#include <mavros_msgs/TrajectoryNominal.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<mavros_msgs::TrajectoryNominal>("mavros/trajectory_nominal/trajectory_nominal_sub",1);

    ros::Rate loop_rate(100);
    
    int count = 0;
    while(ros::ok()){
        mavros_msgs::TrajectoryNominal msg;

        ros::Time t_now = ros::Time::now();
        msg.timestamp = t_now.toNSec();

        for (int i = 0; i < 20; i++) {
            msg.f_out[i] = 1.0f*count;
        }

        traj_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
