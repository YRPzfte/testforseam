#include "common.h"

//生成点云显示器
boost::shared_ptr<pcl::visualization::PCLVisualizer> generate_viewer() {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //设置背景颜色
    viewer->setBackgroundColor (255, 255, 255);
    //设置相机视角宽度
    viewer->setCameraFieldOfView(0.99);
    //设置显示器大小
    viewer->setSize (1560, 820);
    return viewer;
}

//等待点云显示窗口关闭
void wait_cloud_show(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer) {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}


template <typename PointT>
void set_view_point(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, const typename PointCloud<PointT>::Ptr& view_cloud) {
    //计算点云坐标范围，设置观测视点
    PointT min;//用于存放三个轴的最小值
    PointT max;//用于存放三个轴的最大值

    pcl::getMinMax3D(*view_cloud, min, max);

    viewer->setCameraPosition(
        (min.x + max.x) / 2, max.y + (max.y - min.y) * 0.5, max.z + (max.z - min.z) * 2,  // camera位置
        (min.x + max.x) / 2, 0, -1,  // view向量--相机朝向
        0, 0, 0
    );
    viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, 0, 0, 1);
}



void show_3D_pair_point(const PointCloud<PointXYZ>::Ptr& source_cloud, const PointCloud<PointXYZ>::Ptr& target_cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = generate_viewer();

    //将源点云放入场景中
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_cloud, 0, 0, 0); // 设置源点云颜色
    viewer->addPointCloud<pcl::PointXYZ> (source_cloud, source_color, "source cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source cloud");

    //将目标点云放入场景中
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0); // 设置目标点云颜色
    viewer->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target cloud");

    //绘制匹配点对连线
    assert(source_cloud->size() == target_cloud->size());
    for (int i = 0 ; i < source_cloud->size() ; ++i) {
        PointXYZ source_p = source_cloud->points[i];
        PointXYZ target_p = target_cloud->points[i];
        //绘制连线
        ostringstream point_oss;
        point_oss << i << "_point";
        viewer->addLine<pcl::PointXYZ>(source_p, target_p, 0, 238, 0, point_oss.str().c_str());
    }

    //设置观测视点
    PointCloud<PointXYZ>::Ptr view_cloud(new PointCloud<PointXYZ>);
    *view_cloud = *source_cloud + *target_cloud;
    set_view_point<pcl::PointXYZ>(viewer, view_cloud);
    wait_cloud_show(viewer);
}

void show_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = generate_viewer();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");

    //设置观测视点
    set_view_point<pcl::PointXYZRGB>(viewer, cloud);

    wait_cloud_show(viewer);
}

void show_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud, const PointCloud<PointXYZRGB>::Ptr& view_cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = generate_viewer();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");

    //设置观测视点
    set_view_point<pcl::PointXYZRGB>(viewer, view_cloud);

    wait_cloud_show(viewer);
}

PointCloud<PointXYZRGB>::Ptr generate_view_point(const Eigen::Matrix4f& transform, const float& x_offset=0, const float& z_offset=0) {
    Eigen::Matrix4f view_transform = transform;
    view_transform(0, 3) = x_offset;
    view_transform(2, 3) = z_offset;
    PointCloud<PointXYZRGB>::Ptr view_point_cloud(new PointCloud<PointXYZRGB>);
    PointXYZRGB p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    p.b = 0;
    p.g = 0;
    p.r = 255;
    view_point_cloud->points.push_back(p);
    transformPointCloud(*view_point_cloud, *view_point_cloud, view_transform);
    return view_point_cloud;
}

PointCloud<PointXYZRGB>::Ptr generate_view_line(const PointXYZRGB& view_point, const int view_col, int b, int g, int r, const float& farest_z) {
    PointCloud<PointXYZRGB>::Ptr view_line_cloud(new PointCloud<PointXYZRGB>);
    float z = 0;
    while (z + view_point.z > farest_z) {
        for (int row = 0 ; row < img_height * extern_rate ; ++row) {
            PointXYZRGB p;
            p.z = -z;
            p.x = (view_col - img_width * extern_rate / 2) * p.z / fx;
            p.y = (row - img_height * extern_rate / 2) * p.z / fy;
            p.z = -p.z;
            p.b = b;
            p.g = g;
            p.r = r;
            p.x += view_point.x;
            p.y += view_point.y;
            p.z += view_point.z;
            view_line_cloud->points.push_back(p);

        }
        z -= 0.001;
    }
    return view_line_cloud;
}

float cal_farest_z(const PointXYZRGB& source_view_point, const PointXYZRGB& target_view_point, const int& source_col, const int& target_col) {
    float farest_z = 0, source_x, target_x;
    do {
        farest_z -= 0.001;
        source_x = (source_col - img_width * extern_rate / 2) * -farest_z / fx + source_view_point.x;
        target_x = (target_col - img_width * extern_rate / 2) * -farest_z / fx + target_view_point.x;
    } while (source_x <= target_x);
    return farest_z - source_view_point.z;
}

void draw_view_cloud(PointCloud<PointXYZRGB>::Ptr& cloud, const PointXYZRGB& source_view_point, const PointXYZRGB& target_view_point, const int& source_view_col, const int& target_view_col,
                     int source_b, int source_g, int source_r, int target_b, int target_g, int target_r, int overlap_b, int overlap_g, int overlap_r) {

    for (int i = 0 ; i < cloud->size() ; ++i) {
        PointXYZRGB p = cloud->points[i];
        float source_col = (p.x - source_view_point.x)* fx / -(p.z - source_view_point.z) + img_width * extern_rate / 2;
        float target_col = (p.x - target_view_point.x) * fx / -(p.z - target_view_point.z) + img_width * extern_rate / 2;
        if (source_col < source_view_col) {
            float source_row = (p.y - source_view_point.y) * fy / -(p.z - source_view_point.z) + img_height * extern_rate / 2;
            if (int(source_col) % 2 == 0 && int(source_row) % 2 == 0) {
                p.b = source_b;
                p.g = source_g;
                p.r = source_r;
            }
        } else if (target_col > target_view_col) {
            float target_row = (p.y - target_view_point.y) * fy / -(p.z - target_view_point.z) + img_height * extern_rate / 2;
            if (int(target_col) % 2 == 0 && int(target_row) % 2 == 0) {
                p.b = target_b;
                p.g = target_g;
                p.r = target_r;
            }
        } else {
            float overlap_row = (p.y - target_view_point.y) * fy / -(p.z - target_view_point.z) + cy;
            float overlap_col = (p.x - target_view_point.x) * fx / -(p.z - target_view_point.z) + img_width * extern_rate / 2;
            if (int(overlap_row) % 2 == 0 ) {
                p.b = overlap_b;
                p.g = overlap_g;
                p.r = overlap_r;
            }
        }
        cloud->points[i] = p;
    }
}

void show_split_view(const PointCloud<PointXYZRGB>::Ptr& source_cloud, const PointCloud<PointXYZRGB>::Ptr& target_cloud, const Eigen::Matrix4f& source_transform, const Eigen::Matrix4f& target_transform,
                                        const int& source_col, const int& target_col, const float& x_offset, const float& z_offset) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = generate_viewer();

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = x_offset;
    transform(2, 3) = z_offset;
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    transformPointCloud(*source_cloud, *cloud, transform);
    *cloud += *target_cloud;

    PointCloud<PointXYZRGB>::Ptr source_view_point_cloud = generate_view_point(source_transform, x_offset, z_offset);
    PointCloud<PointXYZRGB>::Ptr target_view_point_cloud  = generate_view_point(target_transform);

    PointCloud<PointXYZRGB>::Ptr view_point_cloud(new PointCloud<PointXYZRGB>);
    *view_point_cloud = *source_view_point_cloud + *target_view_point_cloud;

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> view_point_rgb(view_point_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (view_point_cloud, view_point_rgb, "view point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "view point cloud");

    PointXYZRGB source_view_point = source_view_point_cloud->points[0];
    PointXYZRGB target_view_point = target_view_point_cloud->points[0];
    float farest_z = cal_farest_z(source_view_point, target_view_point, source_col, target_col);
    PointCloud<PointXYZRGB>::Ptr source_view_line_cloud = generate_view_line(source_view_point, source_col, 71, 173, 112, farest_z);
    PointCloud<PointXYZRGB>::Ptr target_view_line_cloud = generate_view_line(target_view_point, target_col, 49, 125, 237, farest_z);

    PointCloud<PointXYZRGB>::Ptr view_line_cloud(new PointCloud<PointXYZRGB>);
    *view_line_cloud = *source_view_line_cloud + *target_view_line_cloud;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> view_line_rgb(view_line_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (view_line_cloud, view_line_rgb, "view line cloud");

    draw_view_cloud(cloud, source_view_point, target_view_point, source_col, target_col, 71, 173, 112, 49, 125, 237, 156, 113, 65);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
    //设置观测视点
    set_view_point<pcl::PointXYZRGB>(viewer, cloud);

    wait_cloud_show(viewer);
}

void show_match_distance(const PointCloud<PointXYZ>::Ptr& source_cloud, const PointCloud<PointXYZ>::Ptr& target_cloud, const Eigen::Matrix4f& transform, string label) {
    PointCloud<PointXYZ>::Ptr transform_cloud(new PointCloud<PointXYZ>);
    transformPointCloud(*source_cloud, *transform_cloud, transform);

    assert(transform_cloud->size() == target_cloud->size());
    float distance = 0;
    for (int i = 0 ; i < transform_cloud->size() ; ++i) {
        PointXYZ source_p = transform_cloud->points[i];
        PointXYZ target_p = target_cloud->points[i];
        distance += sqrt((source_p.x - target_p.x) * (source_p.x - target_p.x) + (source_p.y - target_p.y) * (source_p.y - target_p.y) + (source_p.z - target_p.z) * (source_p.z - target_p.z));
    }
    std::cout << label << distance / source_cloud->size() << std::endl;
}
