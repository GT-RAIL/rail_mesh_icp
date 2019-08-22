#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include "rail_mesh_icp/ICPMatch.h"

class ICPMatcher {
    public:
        ICPMatcher(ros::NodeHandle& nh, int iters, float dist, float trans, float fit);
        // handles match point clouds requests
        bool handle_match_clouds_service(rail_mesh_icp::ICPMatch::Request& req, rail_mesh_icp::ICPMatch::Response& res);

    protected:
        ros::NodeHandle matcher_nh_;
        int iters_;
        float dist_;
        float trans_;
        float fit_;
        ros::ServiceServer pose_srv_;
};