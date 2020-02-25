#include "common.h"

RandomSampleConsensus<PointXYZ> compute (const PointCloud<PointXYZ>::Ptr &input,
    const PointCloud<PointXYZ>::Ptr &target,
    Eigen::Matrix4f &transformation,
    const double thresh) {
    SampleConsensusModelRegistration<PointXYZ>::Ptr model (new SampleConsensusModelRegistration<PointXYZ> (input));
    model->setInputTarget (target);

    RandomSampleConsensus<PointXYZ> ransac (model, thresh);
    ransac.setMaxIterations (100000);

    if (!ransac.computeModel(2)) {
        PCL_ERROR ("Could not compute a valid transformation!\n");
        return ransac;
    }
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients (coeff);
    transformation.row (0) = coeff.segment<4>(0);
    transformation.row (1) = coeff.segment<4>(4);
    transformation.row (2) = coeff.segment<4>(8);
    transformation.row (3) = coeff.segment<4>(12);
    return ransac;
}

Eigen::Matrix4f calculate_stitch_transformation(const PointCloud<PointXYZ>::Ptr& source_cloud, const PointCloud<PointXYZ>::Ptr& target_cloud) {
    Eigen::Matrix4f init_transform, transform;
    //粗配准
    double thresh = 0.2;
    RandomSampleConsensus<PointXYZ> ransac = compute(source_cloud, target_cloud, init_transform, thresh);
    show_match_distance(source_cloud, target_cloud, init_transform, "RANSAC Distance:");
    //剔除局外点对
    std::vector<int> inliers;
    ransac.getInliers(inliers);
    std::cout << "Select Inliers Num:" << inliers.size() << std::endl;
    PointCloud<PointXYZ>::Ptr inliers_source_cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr inliers_target_cloud(new PointCloud<PointXYZ>);

    for (int i = 0 ; i < inliers.size() ; ++i) {
        inliers_source_cloud->points.push_back(source_cloud->points[inliers[i]]);
        inliers_target_cloud->points.push_back(target_cloud->points[inliers[i]]);
    }
    //ICP粗配准
    PointCloud<PointXYZ>::Ptr transform_cloud(new PointCloud<PointXYZ>);
    pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(inliers_source_cloud);
    icp.setInputTarget(inliers_target_cloud);
    icp.setTransformationEpsilon(1e-12);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setMaximumIterations(100000);
    /* init_transform为粗略配准算法得到的初始矩阵 */
    icp.align(*transform_cloud, init_transform);
    transform = icp.getFinalTransformation();
    show_match_distance(inliers_source_cloud, inliers_target_cloud, transform, "ICP Distance:");
    return transform;
}