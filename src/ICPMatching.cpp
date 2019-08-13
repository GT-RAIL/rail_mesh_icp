#include "rail_mesh_icp/ICPMatching.h"

ICPMatcher::ICPMatcher(ros::NodeHandle& nh, int iters, float dist, float trans, float fit) {
    matcher_nh_ = nh;
    iters_ = iters;
    dist_ = dist;
    trans_ = trans;
    fit_ = fit;
    pose_srv_ = matcher_nh_.advertiseService("icp_match_clouds", &ICPMatcher::handle_match_clouds_service, this);
}

bool ICPMatcher::handle_match_clouds_service(rail_mesh_icp::ICPMatch::Request& req, rail_mesh_icp::ICPMatch::Response& res) {
    // prepare datastructures
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matched_template_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // loads points clouds
    pcl::fromROSMsg(req.template_cloud,*template_cloud);
    pcl::fromROSMsg(req.target_cloud,*target_cloud);

    // prepare ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
    icp.setInputSource(template_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaximumIterations(iters_);
    icp.setMaxCorrespondenceDistance(dist_);
    icp.setTransformationEpsilon(trans_);
    icp.setEuclideanFitnessEpsilon(fit_);

    // perform ICP to refine template pose
    try {
        icp.align(*matched_template_cloud);
        ROS_INFO("Clouds matched.");
    } catch (...) {
        ROS_ERROR("Could not match point clouds for given params. Please check ICP params.");
        return false;
    }
    Eigen::Matrix4f icp_tf = icp.getFinalTransformation();
    double fitness_score = icp.getFitnessScore();

    // prepares the response to the service request
    pcl::toROSMsg(*matched_template_cloud,res.matched_template_cloud);
    tf::Transform tf_refinement = tf::Transform(tf::Matrix3x3(icp_tf(0,0),icp_tf(0,1),icp_tf(0,2),
                                                                 icp_tf(1,0),icp_tf(1,1),icp_tf(1,2),
                                                                 icp_tf(2,0),icp_tf(2,1),icp_tf(2,2)),
                                                   tf::Vector3(icp_tf(0,3),icp_tf(1,3),icp_tf(2,3)));
    tf::Vector3 chuck_trans = tf_refinement.getOrigin();
    tf::Quaternion chuck_rot = tf_refinement.getRotation();
    res.match_tf.translation.x = chuck_trans.x();
    res.match_tf.translation.y = chuck_trans.y();
    res.match_tf.translation.z = chuck_trans.z();
    res.match_tf.rotation.x = chuck_rot.x();
    res.match_tf.rotation.y = chuck_rot.y();
    res.match_tf.rotation.z = chuck_rot.z();
    res.match_tf.rotation.w = chuck_rot.w();
    res.match_error = fitness_score;
    return true;
}
