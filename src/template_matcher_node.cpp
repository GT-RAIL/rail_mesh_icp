#include <ros/ros.h>

#include "rail_mesh_icp/TemplateMatching.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "template_matcher_node");
    ros::NodeHandle nh, pnh("~");

    // sets the default params
    std::string matching_frame = "map";
    std::string pcl_topic = "/head_camera/depth_registered/points";
    std::string template_file = "corner.pcd";
    std::string initial_estimate_string = "1.9 -0.1 0.83 1.57 0 0";
    std::string template_offset_string = "0 0 0 0 0 0";
    std::string template_frame = "template_pose";
    bool visualize = true;
    bool debug = true;
    bool latched = true;
    bool pre_processed_cloud = false;

    // gets roslaunch params
    pnh.getParam("matching_frame", matching_frame);
    pnh.getParam("pcl_topic", pcl_topic);
    pnh.getParam("template_file", template_file);
    pnh.getParam("initial_estimate_string", initial_estimate_string);
    pnh.getParam("template_offset_string", template_offset_string);
    pnh.getParam("template_frame", template_frame);
    pnh.getParam("visualize", visualize);
    pnh.getParam("debug", debug);
    pnh.getParam("latch_initial", latched);
    pnh.getParam("pre_processed_cloud", pre_processed_cloud);

    // gets the initial_estimate for schunk corner from the launch
    tf::Transform initial_estimate;
    tf::Transform template_offset;

    // initializes a tf for the initial_estimate
    std::vector<float> pose;
    std::istringstream initial_estimate_string_stream(initial_estimate_string);
    for(std::string value_string; initial_estimate_string_stream >> value_string;)
        pose.push_back(std::stof(value_string));
    initial_estimate.setOrigin(tf::Vector3(pose[0],pose[1],pose[2]));
    initial_estimate.setRotation(tf::Quaternion(pose[4],pose[5],pose[3]));

    // initializes a tf for the template_offset
    std::vector<float> offset;
    std::istringstream offset_string_stream(template_offset_string);
    for(std::string value_string; offset_string_stream >> value_string;)
        offset.push_back(std::stof(value_string));
    template_offset.setOrigin(tf::Vector3(offset[0],offset[1],offset[2]));
    template_offset.setRotation(tf::Quaternion(offset[4],offset[5],offset[3]));

    // starts a template matcher
    TemplateMatcher matcher(nh,matching_frame,pcl_topic,template_file,initial_estimate,template_offset,template_frame,
                            visualize,debug,latched,pre_processed_cloud);

    try{
        ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }catch(std::runtime_error& e){
        ROS_ERROR("template_matcher_node exception: %s", e.what());
        return -1;
    }

    return 0;
}
