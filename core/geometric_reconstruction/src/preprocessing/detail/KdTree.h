#pragma once
#include "PreprocessingInternal.h"

namespace reconstruction::detail::preprocessing {
class KdTree {
public:
    explicit KdTree(const std::vector<PointSample>& points) : points_(points) {
        order_.resize(points.size());
        for (std::size_t index = 0; index < order_.size(); ++index) {
            order_[index] = index;
        }
        nodes_.reserve(points.size());
        root_ = build(0U, order_.size(), 0);
    }

    std::vector<std::pair<double, std::size_t>> nearest(
        std::size_t queryIndex,
        std::size_t count) const {
        Heap heap;
        search(root_, points_[queryIndex].position, queryIndex, count, heap);
        std::vector<std::pair<double, std::size_t>> result;
        result.reserve(heap.size());
        while (!heap.empty()) {
            result.emplace_back(std::get<0>(heap.top()), std::get<2>(heap.top()));
            heap.pop();
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

private:
    struct Node {
        std::size_t pointIndex = 0;
        int axis = 0;
        int left = -1;
        int right = -1;
    };

    using Heap = std::priority_queue<std::tuple<double, PointId, std::size_t>>;

    int build(std::size_t begin, std::size_t end, int depth) {
        if (begin >= end) {
            return -1;
        }
        const int axis = depth % 3;
        const std::size_t middle = begin + (end - begin) / 2U;
        std::nth_element(
            order_.begin() + static_cast<std::ptrdiff_t>(begin),
            order_.begin() + static_cast<std::ptrdiff_t>(middle),
            order_.begin() + static_cast<std::ptrdiff_t>(end),
            [this, axis](std::size_t left, std::size_t right) {
                return std::make_pair(coordinate(points_[left].position, axis), points_[left].id) <
                    std::make_pair(coordinate(points_[right].position, axis), points_[right].id);
            });
        const int nodeIndex = static_cast<int>(nodes_.size());
        nodes_.push_back(Node{order_[middle], axis, -1, -1});
        const int left = build(begin, middle, depth + 1);
        const int right = build(middle + 1U, end, depth + 1);
        nodes_[static_cast<std::size_t>(nodeIndex)].left = left;
        nodes_[static_cast<std::size_t>(nodeIndex)].right = right;
        return nodeIndex;
    }

    void search(
        int nodeIndex,
        const Vec3& query,
        std::size_t excludedIndex,
        std::size_t count,
        Heap& heap) const {
        if (nodeIndex < 0 || count == 0U) {
            return;
        }
        const Node& node = nodes_[static_cast<std::size_t>(nodeIndex)];
        const Vec3& point = points_[node.pointIndex].position;
        if (node.pointIndex != excludedIndex) {
            const double distance = squaredDistance(query, point);
            const auto candidate = std::make_tuple(distance, points_[node.pointIndex].id, node.pointIndex);
            if (heap.size() < count) {
                heap.push(candidate);
            } else if (candidate < heap.top()) {
                heap.pop();
                heap.push(candidate);
            }
        }

        const double delta = coordinate(query, node.axis) -
            coordinate(point, node.axis);
        const int nearChild = delta < 0.0 ? node.left : node.right;
        const int farChild = delta < 0.0 ? node.right : node.left;
        search(nearChild, query, excludedIndex, count, heap);
        if (heap.size() < count || delta * delta <= std::get<0>(heap.top())) {
            search(farChild, query, excludedIndex, count, heap);
        }
    }

    const std::vector<PointSample>& points_;
    std::vector<std::size_t> order_;
    std::vector<Node> nodes_;
    int root_ = -1;
};
} // namespace reconstruction::detail::preprocessing
