# -*- coding: utf-8 -*-
"""最基础的 K-Means 聚类算法实现（纯 Python，无第三方依赖）"""
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

def kmeans(data_points, num_clusters, max_iters=100, seed=0):
    """
    K-Means 主函数

    参数:
        data_points: 样本列表，每个样本是一个向量（如 [x, y]）
        num_clusters: 聚类中心个数
        max_iters: 最大迭代次数（防止不收敛时死循环）
        seed: 随机种子，保证结果可复现
    返回:
        cluster_centers: 最终聚类中心（num_clusters 个向量）
        cluster_labels:  每个样本所属簇的编号（与 data_points 等长的列表）
    """
    rng = random.Random(seed)

    # ---------- 1. 初始化：随机选 num_clusters 个样本作为初始中心 ----------
    sampled_points = rng.sample(data_points, num_clusters)
    cluster_centers = []
    for point in sampled_points:
        cluster_centers.append(list(point))

    for _ in range(max_iters):
        # ---------- 2. 分配步骤：把每个样本分给最近的中心 ----------
        cluster_labels = []  # cluster_labels[point_idx] = 样本 data_points[point_idx] 所属簇的编号
        for point in data_points:
            # 先假设 0 号中心最近，再把其余中心逐个比一遍
            nearest_cluster_idx = 0
            nearest_dist = squared_distance(point, cluster_centers[0])
            for cluster_idx in range(1, num_clusters):
                dist = squared_distance(point, cluster_centers[cluster_idx])
                if dist < nearest_dist:
                    nearest_cluster_idx = cluster_idx
                    nearest_dist = dist
            cluster_labels.append(nearest_cluster_idx)

        # ---------- 3. 更新步骤：把每个中心移动到其成员的平均位置 ----------
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
                new_center = cluster_centers[cluster_idx]  # 空簇保留原中心，避免除零
            new_centers.append(new_center)

        # ---------- 4. 收敛判断：中心不再变化就提前结束 ----------
        converged = True  # 先假设已收敛，发现任何一维还在动就推翻
        for old_center, new_center in zip(cluster_centers, new_centers):
            for old_coordinate, new_coordinate in zip(old_center, new_center):
                if abs(old_coordinate - new_coordinate) >= 1e-12:
                    converged = False
        if converged:
            break
        cluster_centers = new_centers

    return cluster_centers, cluster_labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点
    rng = random.Random(42)
    data_points = []
    for _ in range(100):
        data_points.append([rng.gauss(0, 0.5), rng.gauss(0, 0.5)])  # 中心 (0,0)
    for _ in range(100):
        data_points.append([rng.gauss(5, 0.5), rng.gauss(5, 0.5)])  # 中心 (5,5)
    for _ in range(100):
        data_points.append([rng.gauss(10, 0.5), rng.gauss(10, 0.5)])  # 中心 (10,0)

    cluster_centers, cluster_labels = kmeans(data_points, num_clusters=3)

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
