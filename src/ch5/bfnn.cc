//
// Created by xiang on 2021/8/18.
//

#include "ch5/bfnn.h"
#include <execution>

namespace sad {

// 找单个点的最近邻
int bfnn_point(CloudPtr cloud, const Vec3f& point) {
    // 返回最大距离点云的序号, 所以要减去begin()
    return std::min_element(cloud->points.begin(), cloud->points.end(),
                            [&point](const PointType& pt1, const PointType& pt2) -> bool {
                                return (pt1.getVector3fMap() - point).squaredNorm() <
                                       (pt2.getVector3fMap() - point).squaredNorm();
                            }) -
           cloud->points.begin();
}

// 找单个点的knn
std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point, int k) {
    struct IndexAndDis {
        IndexAndDis() {}
        IndexAndDis(int index, double dis2) : index_(index), dis2_(dis2) {}
        int index_ = 0;
        double dis2_ = 0;
    };

    // 计算点云中所有点到point的距离
    std::vector<IndexAndDis> index_and_dis(cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        index_and_dis[i] = {i, (cloud->points[i].getVector3fMap() - point).squaredNorm()};
    }
    std::sort(index_and_dis.begin(), index_and_dis.end(),
              [](const auto& d1, const auto& d2) { return d1.dis2_ < d2.dis2_; });
    std::vector<int> ret;
    // back_inserter: 向ret末尾添加元素
    std::transform(index_and_dis.begin(), index_and_dis.begin() + k, std::back_inserter(ret),
                   [](const auto& d1) { return d1.index_; });
    return ret;
}

// 多线程 找cloud2的最近邻
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    // 先生成索引
    std::vector<size_t> index(cloud1->size());
    // mutable表示lambda表达式可以修改它所捕获的变量idx的值
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    // 并行化for_each
    matches.resize(index.size());
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    // 单线程版本
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    matches.resize(index.size());
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

void bfnn_cloud_mt_k(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches, int k) {
    // 先生成索引
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    // 并行化for_each
    matches.resize(index.size() * k);
    // https://qa.1r1g.com/sf/ask/2796827491/
    // seq 表示顺序执行, 与不执行策略相同
    // par 表示并行执行, 您有责任确保内部不会发生任何数据争用
    // par_unseq 除了允许在多个线程中执行之外,还允许实现在单个线程内交错各个循环迭代
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        auto v = bfnn_point_k(cloud1, ToVec3f(cloud2->points[idx]), k);
        for (int i = 0; i < v.size(); ++i) {
            matches[idx * k + i].first = v[i];
            matches[idx * k + i].second = idx;
        }
    });
}

}  // namespace sad
