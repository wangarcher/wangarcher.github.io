# -*- coding: utf-8 -*-
"""二分 K-Means（Bisecting K-Means）：从"所有点算一个簇"出发，
每轮挑误差最大的簇用基础 K-Means(k=2) 一分为二，直到凑够目标簇数。
相比一次性划分，通常能得到更稳定、更接近全局最优的结果"""
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
    """基础 K-Means（与 k_means.py 相同），供二分时调用"""
    rng = random.Random(seed)
    sampled_points = rng.sample(data_points, num_clusters)
    cluster_centers = []
    for point in sampled_points:
        cluster_centers.append(list(point))

    for _ in range(max_iters):
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

        members = []
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
                new_center = cluster_centers[cluster_idx]
            new_centers.append(new_center)

        converged = True
        for old_center, new_center in zip(cluster_centers, new_centers):
            for old_coordinate, new_coordinate in zip(old_center, new_center):
                if abs(old_coordinate - new_coordinate) >= 1e-12:
                    converged = False

        if converged:
            break
        cluster_centers = new_centers

    return cluster_centers, cluster_labels


def cluster_sse(points):
    """一个簇的误差平方和（SSE）：各成员到簇质心距离的平方之和，越大说明这个簇分得越差"""
    center = compute_mean(points)
    error_sum = 0.0
    for point in points:
        error_sum += squared_distance(point, center)
    return error_sum


def bisecting_k_means(data_points, num_clusters, seed=0):
    """
    二分 K-Means 主函数

    参数:
        data_points: 样本列表，每个样本是一个向量（如 [x, y]）
        num_clusters: 目标簇数
        seed: 随机种子，保证结果可复现
    返回:
        cluster_centers: 最终聚类中心（num_clusters 个向量）
        cluster_labels:  每个样本所属簇的编号（与 data_points 等长的列表）
    """
    rng = random.Random(seed)
    # clusters[j] 存第 j 簇成员在 data_points 中的下标，初始为"全体样本一个簇"
    clusters = [list(range(len(data_points)))]

    # ---------- 1. 反复二分，直到凑够 num_clusters 个簇 ----------
    while len(clusters) < num_clusters:
        # 挑 SSE 最大的簇来分裂：误差最大 = 当前分得最差
        worst_cluster_idx = 0
        worst_cluster_error = 0
        for cluster_idx, member_indices in enumerate(clusters):
            cluster_points = []
            for point_idx in member_indices:
                cluster_points.append(data_points[point_idx])
            cluster_error = cluster_sse(cluster_points)
            if cluster_error > worst_cluster_error:
                worst_cluster_idx = cluster_idx
                worst_cluster_error = cluster_error

        member_indices = clusters[worst_cluster_idx]

        # ---------- 2. 用基础 K-Means 把这个簇分成两半 ----------
        sub_points = []
        for point_idx in member_indices:
            sub_points.append(data_points[point_idx])
        split_successfully = False
        for _ in range(10):  # 极端情况（点重合）会分不出两半，换随机种子重试
            _, sub_labels = kmeans(sub_points, num_clusters=2, seed=rng.randint(0, 10**9))
            first_half = []
            second_half = []
            for point_idx, label in zip(member_indices, sub_labels):
                if label == 0:
                    first_half.append(point_idx)
                elif label == 1:
                    second_half.append(point_idx)
            if first_half and second_half:
                split_successfully = True
                break

        if not split_successfully:
            raise ValueError("簇内样本完全相同，无法继续二分")

        # ---------- 3. 用两个新簇替换被分裂的簇 ----------
        clusters[worst_cluster_idx:worst_cluster_idx + 1] = [first_half, second_half]

    # ---------- 4. 汇总：各簇质心 + 与输入顺序对齐的簇编号 ----------
    cluster_centers = []
    for member_indices in clusters:
        cluster_points = []
        for point_idx in member_indices:
            cluster_points.append(data_points[point_idx])
        cluster_center = compute_mean(cluster_points)
        cluster_centers.append(cluster_center)

    cluster_labels = []
    for _ in range(len(data_points)):
        cluster_labels.append(None)
    for cluster_idx, member_indices in enumerate(clusters):
        for point_idx in member_indices:
            cluster_labels[point_idx] = cluster_idx

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

    cluster_centers, cluster_labels = bisecting_k_means(data_points, num_clusters=3)

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
