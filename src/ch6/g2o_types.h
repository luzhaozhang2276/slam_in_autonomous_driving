//
// Created by xiang on 2022/3/22.
//

#ifndef SLAM_IN_AUTO_DRIVING_G2O_TYPES_H
#define SLAM_IN_AUTO_DRIVING_G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <utility>
#include <pcl/search/kdtree.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"

namespace sad {

class VertexSE2 : public g2o::BaseVertex<3, SE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override { _estimate = SE2(); }
    void oplusImpl(const double* update) override {
        _estimate.translation()[0] += update[0];
        _estimate.translation()[1] += update[1];
        _estimate.so2() = _estimate.so2() * SO2::exp(update[2]);
    }

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }
};

class EdgeSE2LikelihoodFiled : public g2o::BaseUnaryEdge<1, double, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE2LikelihoodFiled(const cv::Mat& field_image, double range, double angle, float resolution = 10.0)
        : field_image_(field_image), range_(range), angle_(angle), resolution_(resolution) {}

    /// 判定此条边是否在field image外面
    bool IsOutSide() {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        Vec2d pw = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pf = (pw * resolution_ + Vec2d(field_image_.rows / 2, field_image_.cols / 2)).cast<int>();  // 图像坐标

        if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ && pf[1] >= image_boarder_ &&
            pf[1] < field_image_.rows - image_boarder_) {
            return false;
        } else {
            return true;
        }
    }

    void computeError() override {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        Vec2d pw = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2d pf =
            pw * resolution_ + Vec2d(field_image_.rows / 2, field_image_.cols / 2) - Vec2d(0.5, 0.5);  // 图像坐标

        if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ && pf[1] >= image_boarder_ &&
            pf[1] < field_image_.rows - image_boarder_) {
            _error[0] = math::GetPixelValue<float>(field_image_, pf[0], pf[1]);
        } else {
            _error[0] = 0;
            setLevel(1);
        }
    }

    void linearizeOplus() override {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        float theta = pose.so2().log();
        Vec2d pw = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2d pf =
            pw * resolution_ + Vec2d(field_image_.rows / 2, field_image_.cols / 2) - Vec2d(0.5, 0.5);  // 图像坐标

        if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ && pf[1] >= image_boarder_ &&
            pf[1] < field_image_.rows - image_boarder_) {
            // 图像梯度
            float dx = 0.5 * (math::GetPixelValue<float>(field_image_, pf[0] + 1, pf[1]) -
                              math::GetPixelValue<float>(field_image_, pf[0] - 1, pf[1]));
            float dy = 0.5 * (math::GetPixelValue<float>(field_image_, pf[0], pf[1] + 1) -
                              math::GetPixelValue<float>(field_image_, pf[0], pf[1] - 1));

            _jacobianOplusXi << resolution_ * dx, resolution_ * dy,
                -resolution_ * dx * range_ * std::sin(angle_ + theta) +
                    resolution_ * dy * range_ * std::cos(angle_ + theta);
        } else {
            _jacobianOplusXi.setZero();
            setLevel(1);
        }
    }

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }

   private:
    const cv::Mat& field_image_;
    double range_ = 0;
    double angle_ = 0;
    float resolution_ = 10.0;
    inline static const int image_boarder_ = 10;
};

class EdgeSE2Point2Point : public g2o::BaseUnaryEdge<2, Vec2d, VertexSE2>{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit EdgeSE2Point2Point(double angle, float r) :_r(r), _angle(angle) {}

    void computeError() override {
        auto* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        Vec2d pw = pose * Vec2d(_r * std::cos(_angle), _r * std::sin(_angle));
//        Vec2d m_pt(_target_cloud->points[_nn_idx[0]].x, _target_cloud->points[_nn_idx[0]].y);
//        setMeasurement(m_pt);

        _error =  pw - _measurement;
    }

    void linearizeOplus() override {
        auto* v = (VertexSE2*)_vertices[0];
        double theta = v->estimate().so2().log();
        _jacobianOplusXi << 1, 0, 0, 1, -_r * std::sin(_angle + theta), _r * std::cos(_angle + theta);
    }

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }

   protected:
    double _angle;
    float _r;
};

class EdgeSE2Point2Plane : public g2o::BaseUnaryEdge<1, double, VertexSE2>{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit EdgeSE2Point2Plane(const Vec3d line_coeffs, double angle, float r)
        :_r(r), _angle(angle), _line_coeffs(line_coeffs) {}

    void computeError() override {
        auto* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        Vec2d pw = pose * Vec2d(_r * std::cos(_angle), _r * std::sin(_angle));

        _error[0] =  _line_coeffs[0] * pw[0] + _line_coeffs[1] * pw[1] + _line_coeffs[2];
    }

    void linearizeOplus() override {
        auto* v = (VertexSE2*)_vertices[0];
        double theta = v->estimate().so2().log();
        _jacobianOplusXi << _line_coeffs[0], _line_coeffs[1],
            -_line_coeffs[0] * _r * std::sin(_angle + theta) + _line_coeffs[1] * _r * std::cos(_angle + theta);
    }

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }

   protected:
    double _angle;
    float _r;
    Vec3d _line_coeffs;
};

/**
 * SE2 pose graph使用
 * error = v1.inv * v2 * meas.inv
 */
class EdgeSE2 : public g2o::BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE2() {}

    void computeError() override {
        VertexSE2* v1 = (VertexSE2*)_vertices[0];
        VertexSE2* v2 = (VertexSE2*)_vertices[1];
        _error = (v1->estimate().inverse() * v2->estimate() * measurement().inverse()).log();
    }

    // TODO jacobian

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }

   private:
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_G2O_TYPES_H