# -*- coding: utf-8 -*-
"""K-Means++：只改进基础 K-Means 的"初始中心怎么挑"，之后的迭代完全相同。
初始中心彼此散开而不是扎堆，通常收敛更快、更不容易落入差的局部最优"""
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方（不开方也能比较远近）"""
    distance = 0.0
    for coordinate_a, coordinate_b in zip(point_a, point_b):
        difference = coordinate_a - coordinate_b
        distance += difference * difference
    return distance


def compute_mean(points):
    """一组点的均值（逐维取平均）"""
    num_points = len(points)
    num_dims = len(points[0])
    mean = []
    for dimension_idx in range(num_dims):
        dimension_sum = 0.0
        for point in points:
            dimension_sum += point[dimension_idx]
        mean.append(dimension_sum / num_points)
    return mean


def plus_plus_init(data_points, num_clusters, rng):
    """
    K-Means++ 初始化

    规则：第 1 个中心纯随机；之后每个新中心按 D(x)^2 加权抽取——
    离已有中心越远的点，被选为新中心的概率越大
    """
    centers = []
    first_center = list(rng.choice(data_points))  # 第 1 个中心：纯随机
    centers.append(first_center)

    while len(centers) < num_clusters:
        # 每个点到"离它最近的已有中心"的距离平方，作为被抽中的权重
        weights = []
        for point in data_points:
            nearest_distance = squared_distance(point, centers[0])
            for center in centers[1:]:
                distance = squared_distance(point, center)
                if distance < nearest_distance:
                    nearest_distance = distance
            weights.append(nearest_distance)
        centers.append(list(rng.choices(data_points, weights=weights, k=1)[0]))

    return centers


def k_means_plus_plus(data_points, num_clusters, max_iters=100, seed=0):
    """
    K-Means++ 主函数

    参数与返回值和基础 K-Means 完全一致，唯一区别是初始中心的挑选方式
    """
    rng = random.Random(seed)

    # ---------- 1. 初始化：K-Means++ 挑选彼此散开的初始中心 ----------
    cluster_centers = plus_plus_init(data_points, num_clusters, rng)

    # ---------- 2~4. 分配 / 更新 / 收敛判断：与基础 K-Means 相同 ----------
    for _ in range(max_iters):
        # 分配步骤：把每个样本分给最近的中心
        cluster_labels = []
        for point in data_points:
            nearest_cluster_idx = 0
            nearest_distance = squared_distance(point, cluster_centers[0])
            for cluster_idx in range(1, num_clusters):
                distance = squared_distance(point, cluster_centers[cluster_idx])
                if distance < nearest_distance:
                    nearest_cluster_idx = cluster_idx
                    nearest_distance = distance
            cluster_labels.append(nearest_cluster_idx)

        # 更新步骤：把每个中心移动到其成员的平均位置
        members = []  # members[j] 为第 j 簇的全部样本
        for _ in range(num_clusters):
            members.append([])
        for point, label in zip(data_points, cluster_labels):
            members[label].append(point)
        new_centers = []
        for cluster_idx in range(num_clusters):
            cluster_points = members[cluster_idx]
            if cluster_points:
                new_center = compute_mean(cluster_points)
            else:
                new_center = cluster_centers[cluster_idx]  # 空簇保留原中心
            new_centers.append(new_center)

        # 收敛判断：中心不再变化就提前结束
        converged = True
        for old_center, new_center in zip(cluster_centers, new_centers):
            for old_coordinate, new_coordinate in zip(old_center, new_center):
                if abs(old_coordinate - new_coordinate) >= 1e-12:
                    converged = False

        if converged:
            break
        cluster_centers = new_centers

    return cluster_centers, cluster_labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点（与 k_means.py 相同）
    rng = random.Random(42)
    data_points = []
    for _ in range(100):
        data_points.append([rng.gauss(0, 0.5), rng.gauss(0, 0.5)])  # 中心 (0,0)
    for _ in range(100):
        data_points.append([rng.gauss(5, 0.5), rng.gauss(5, 0.5)])  # 中心 (5,5)
    for _ in range(100):
        data_points.append([rng.gauss(10, 0.5), rng.gauss(10, 0.5)])  # 中心 (10,0)

    cluster_centers, cluster_labels = k_means_plus_plus(data_points, num_clusters=3)

    print("聚类中心:")
    for center in cluster_centers:
        rounded_center = []
        for value in center:
            rounded_center.append(round(value, 3))
        print(rounded_center)
    cluster_sizes = []
    for cluster_idx in range(3):
        cluster_sizes.append(cluster_labels.count(cluster_idx))
    print("各簇样本数:", cluster_sizes)
