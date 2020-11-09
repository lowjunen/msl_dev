#include <ros/ros.h>
#include <mavros_msgs/TrajectoryNominal.h>

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define MAXBUFSIZE ((int) 1e6)

MatrixXd readMatrix(const char *filename)
{
    int cols = 0, row = 0;
    double buff[MAXBUFSIZE];

    ifstream infile;
    infile.open(filename);
    while (! infile.eof())
    {
        string line;
        getline(infile,line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];
            
        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;
        
        rows++;
    }
    infile.close();

    rows--;

    MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];
    return result;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<mavros_msgs::TrajectoryNominal>("mavros/trajectory_nominal/trajectory_nominal_sub",1);

    ros::Rate loop_rate(100);
    
    MatrixXd test = readMatrix("f_out_hover.csv");

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
