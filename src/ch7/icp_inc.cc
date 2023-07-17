//
// Created by xiang on 2022/7/7.
//

#include "icp_inc.h"
#include "common/math_utils.h"

#include <execution>
#include <iostream>

namespace sad {

void IncIcp3d::ComputeResidualAndJacobians(const SE3& input_pose, Mat18d& HTVH,
                                           Vec18d& HTVr) {
    assert(target_ != nullptr && source_ != nullptr);
    SE3 pose = input_pose;

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 1, 18>> jacobians(index.size());
    std::vector<double> errors(index.size());

    // gauss-newton 迭代
    // 最近邻，可以并发
    std::for_each(
        std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 转换之后的q
            // std::vector<int> nn;
            PointVec nn;
            ikdtree_ptr->GetClosestPoint(ToPointType(qs), nn,
                                         5);  // 这里取5个最近邻
            if (nn.size() > 3) {
                // convert to eigen
                std::vector<Vec3d> nn_eigen;
                for (int i = 0; i < nn.size(); ++i) {
                    nn_eigen.emplace_back(ToVec3d(nn[i]));
                }

                Vec4d n;
                if (!math::FitPlane(nn_eigen, n)) {
                    // 失败的不要
                    effect_pts[idx] = false;
                    return;
                }

                double dis = n.head<3>().dot(qs) + n[3];
                if (fabs(dis) > options_.max_plane_distance_) {
                    // 点离的太远了不要
                    effect_pts[idx] = false;
                    return;
                }

                effect_pts[idx] = true;

                // build residual
                Eigen::Matrix<double, 1, 18> J;
                J.setZero();
                J.block<1, 3>(0, 0) = n.head<3>().transpose();
                J.block<1, 3>(0, 6) = -n.head<3>().transpose() *
                                      pose.so3().matrix() * SO3::hat(q);

                jacobians[idx] = J;
                errors[idx] = dis;
            } else {
                effect_pts[idx] = false;
            }
        });

    // 累加Hessian和error,计算dx
    double total_res = 0;
    int effective_num = 0;

    HTVH.setZero();
    HTVr.setZero();

    const double info_ratio = 10.0;  // 每个点反馈的info因子

    for (int idx = 0; idx < effect_pts.size(); ++idx) {
        if (!effect_pts[idx]) {
            continue;
        }

        total_res += errors[idx] * errors[idx];
        effective_num++;

        HTVH += jacobians[idx].transpose() * jacobians[idx] * info_ratio;
        HTVr += -jacobians[idx].transpose() * errors[idx] * info_ratio;
    }

    LOG(INFO) << "effective: " << effective_num;
    std::cout << HTVH.block<9, 9>(0, 0) << std::endl;
}

void IncIcp3d::BuildTargetKdTree() {
    if (ikdtree_ptr == nullptr) {
        ikdtree_ptr = std::make_shared<
            KD_TREE<sad::PointType>>();  // ikdtree为空，构建ikdtree
        ikdtree_ptr->Build(target_->points);
    } else {
        ikdtree_ptr->Add_Points(
            target_->points,
            false);  // 当ikdtree非空时，增量的插入点云，不再进行降采样
    }
}

}  // namespace sad