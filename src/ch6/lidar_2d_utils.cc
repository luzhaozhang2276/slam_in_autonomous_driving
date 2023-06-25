//
// Created by xiang on 2022/3/15.
//

#include "ch6/lidar_2d_utils.h"
#include <opencv2/imgproc.hpp>

#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

#include "common/eigen_types.h"
#include "common/math_utils.h"

namespace sad {

void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap) {
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }

        double real_angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(x, y));

        int image_x = int(psubmap[0] * resolution + image_size / 2);
        int image_y = int(psubmap[1] * resolution + image_size / 2);
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
        }
    }

    // 同时画出pose自身所在位置
    Vec2d pose_in_image =
        pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);
}

void DetectDegraded(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                    float resolution, const SE2& pose_submap) {
    static int idx = 0;
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    // 点云坐标转换：极坐标 -> 笛卡尔坐标
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }

        double real_angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        cloud->push_back(pcl::PointXYZ(x, y, 0.0));
    }

    // 点云着色与可视化
    auto DrawColor = [&](pcl::PointXYZ point, Vec3b color) {
        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(point.x, point.y));
        int image_x = int(psubmap[0] * resolution + image_size / 2);
        int image_y = int(psubmap[1] * resolution + image_size / 2);
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
        }
    };

    // 计算叉乘
    auto CrossProductNorm = [](Vec3d vec1, Vec3d vec2) {
        // 计算叉乘积
        Vec3d result;
        result[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
        result[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
        result[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];

        return result.norm();
    };

    for (const auto& p : cloud->points) DrawColor(p, color);  // 默认颜色: 蓝色

    LOG(INFO) << "idx: " << idx;
    LOG(INFO) << "point cloud size: " << cloud->size();
    bool b_degraded = true;
    {
        // 创建 KD-Tree 设置参数并执行 Euclidean 聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);  // 设置距离阈值
        ec.setMinClusterSize(50);     // 设置簇的最小大小
        ec.setMaxClusterSize(1500);   // 设置簇的最大大小
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 输出聚类结果
        std::vector<Vec3b> color_map;
        color_map.push_back(Vec3b(0, 0, 255));
        color_map.push_back(Vec3b(0, 255, 0));
        color_map.push_back(Vec3b(0, 255, 255));
        color_map.push_back(Vec3b(255, 0, 255));
        color_map.push_back(Vec3b(255, 255, 0));
        LOG(INFO) << "Number of clusters found: " << cluster_indices.size();
        int line_cnt = 0;
        std::string cluster_info = "Cluster size: " + std::to_string(cluster_indices.size());
        cv::putText(image, cluster_info, cv::Point(260, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        std::vector<Vec3d> line_dirs;
        for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
            LOG(INFO) << "Cluster " << i << " has " << cluster_indices[i].indices.size() << " points.";
            Vec3b draw_color = Vec3b(0, 125, 125);
            if (i <= color_map.size()) draw_color = color_map[i];

            std::vector<Vec3d> line_points;
            for (const auto& id : cluster_indices[i].indices) {
                DrawColor(cloud->points[id], draw_color);
                line_points.push_back(Vec3d(cloud->points[id].x, cloud->points[id].y, 0.0));
            }

            // 打印聚类结果
            cluster_info =
                "i: " + std::to_string(i) + " [" + std::to_string(cluster_indices[i].indices.size()) + " points]";
            cv::putText(image, cluster_info, cv::Point(260, 50 + i * 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 0, 255), 1);

            // 进行直线拟合
            Vec3d esti_origin, esti_dir;
            std::string line_info = "FitLine failed.";
            if (sad::math::FitLine(line_points, esti_origin, esti_dir, 0.5)) {
                // 绘制直线
                line_dirs.push_back(esti_dir);
                line_cnt++;

                // 顶点坐标转换
                Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(esti_origin.x(), esti_origin.y()));
                // Vec2d dir = pose_submap.inverse() * (pose * Vec2d(esti_dir.x(), esti_dir.y()));
                int image_x = int(psubmap[0] * resolution + image_size / 2);
                int image_y = int(psubmap[1] * resolution + image_size / 2);

                // Define line vertex and direction vector.
                cv::Point2f vertex(image_x, image_y);
                cv::Vec2f direction(esti_dir.x(), esti_dir.y());

                // Compute line start and end points.
                float length = 200;
                cv::Vec2f unit_direction = direction / cv::norm(direction);
                cv::Point2f pt1(vertex - length / 2 * cv::Point2f(unit_direction[0], unit_direction[1]));
                cv::Point2f pt2(vertex + length / 2 * cv::Point2f(unit_direction[0], unit_direction[1]));

                // Draw line on image.
                cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 1);
                cv::putText(image, std::to_string(i), cv::Point(image_x - 5, image_y - 5), cv::FONT_HERSHEY_SIMPLEX,
                            0.5, cv::Scalar(0, 0, 0), 1);

                // line info
                line_info = "FitLine successed.";
                cv::putText(image, line_info, cv::Point(430, 50 + i * 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 255, 0), 1);
            } else
                cv::putText(image, line_info, cv::Point(430, 50 + i * 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 0, 255), 1);
        }
        LOG(INFO) << "Number of line found: " << line_cnt;
        std::string lines_info = "lines size: " + std::to_string(line_cnt);
        cv::putText(image, lines_info, cv::Point(430, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        // 判断退化状态
        double max_score = 0.0;
        if (cluster_indices.empty()) b_degraded = true;  // 退化：空旷场景
        else if (line_dirs.size() == cluster_indices.size()) {
            if (line_dirs.size() == 1) b_degraded = true;  // 退化：空旷场景
            else {
                // 方向向量两两叉乘，判断是否平行
                for (size_t i = 0; i < line_dirs.size(); ++i) {
                    for (size_t j = i + 1; j < line_dirs.size(); ++j) {
                        double score = CrossProductNorm(line_dirs[i], line_dirs[j]);
                        max_score = std::max(max_score, score);
                        LOG(INFO) << i << " * " << j << ", score: " << score;
                    }
                }
                // 根据叉乘得分最大值判断是否退化
                b_degraded = max_score < 0.3 ? true : false;
            }
        } else
            b_degraded = false;  // 非退化：存在无法拟合直线的点云团簇，复杂场景

        std::string score_info = "Score: " + std::to_string(max_score);
        cv::putText(image, score_info, cv::Point(600, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    // 同时画出pose自身所在位置
    Vec2d pose_in_image =
        pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);

    cv::putText(image, "id: " + std::to_string(idx++), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2);

    if (b_degraded)
        cv::putText(image, "error: degraded scene", cv::Point(600, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1);
    else
        cv::putText(image, "info: normal scene", cv::Point(600, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);
}

}  // namespace sad