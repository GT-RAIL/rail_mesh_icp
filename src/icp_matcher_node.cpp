#include <ros/ros.h>
#include "rail_mesh_icp/ICPMatching.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "icp_matcher_node");
    ros::NodeHandle nh;

    // loads ICP params
    int iters = 50;
    float dist = 1.0;
    float trans = 1e-8;
    float fit = 1;
    nh.getParam("/icp_matcher_node/iterations",iters);
    nh.getParam("/icp_matcher_node/max_distance",dist);
    nh.getParam("/icp_matcher_node/trans_epsilon",trans);
    nh.getParam("/icp_matcher_node/fit_epsilon",fit);

    // start the ICP matcher
    ICPMatcher matcher(nh,iters,dist,trans,fit);

    try{
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("icp_matcher_node exception: %s", e.what());
        return -1;
    }

    return 0;
}