# -*- coding: utf-8 -*-
"""Spherical K-Means（球面 K-Means）：先把所有样本归一化到单位长度，
用余弦相似度（= 单位向量的点积）代替欧氏距离来聚类。
衡量的是"方向"而非"远近"，常用于文本聚类（TF-IDF 文档向量的方向代表主题）"""
import math
import random


def dot_product(point_a, point_b):
    """两向量的点积"""
    result = 0.0
    for coordinate_a, coordinate_b in zip(point_a, point_b):
        result += coordinate_a * coordinate_b
    return result


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


def normalize(point):
    """归一化到单位长度：模长变为 1，只保留方向"""
    squared_length = 0.0
    for coordinate in point:
        squared_length += coordinate * coordinate
    length = math.sqrt(squared_length)
    normalized = []
    for coordinate in point:
        normalized.append(coordinate / length)
    return normalized


def spherical_k_means(data_points, num_clusters, max_iters=100, seed=0):
    """
    Spherical K-Means 主函数

    参数:
        data_points: 样本列表，每个样本是一个向量（如 [x, y]）
        num_clusters: 聚类中心个数
        max_iters: 最大迭代次数（防止不收敛时死循环）
        seed: 随机种子，保证结果可复现
    返回:
        cluster_centers: 最终聚类中心（单位向量，代表各簇的"主方向"）
        cluster_labels:  每个样本所属簇的编号（与 data_points 等长的列表）
    """
    rng = random.Random(seed)

    # ---------- 1. 预处理：所有样本归一化到单位长度，只保留方向 ----------
    unit_points = []
    for point in data_points:
        unit_points.append(normalize(point))

    # ---------- 2. 初始化：随机选 num_clusters 个单位向量作为初始中心 ----------
    sampled_points = rng.sample(unit_points, num_clusters)
    cluster_centers = []
    for point in sampled_points:
        cluster_centers.append(list(point))

    for _ in range(max_iters):
        # ---------- 3. 分配步骤：余弦相似度最大 = 单位向量点积最大 ----------
        cluster_labels = []
        for point in unit_points:
            nearest_cluster_idx = 0
            highest_similarity = dot_product(point, cluster_centers[0])
            for cluster_idx in range(1, num_clusters):
                similarity = dot_product(point, cluster_centers[cluster_idx])
                if similarity > highest_similarity:
                    nearest_cluster_idx = cluster_idx
                    highest_similarity = similarity
            cluster_labels.append(nearest_cluster_idx)

        # ---------- 4. 更新步骤：中心取成员均值后重新归一化（拉回单位球面） ----------
        members = []  # members[j] 为第 j 簇的全部样本
        for _ in range(num_clusters):
            members.append([])
        for point, label in zip(unit_points, cluster_labels):
            members[label].append(point)
        new_centers = []
        for cluster_idx, cluster_points in enumerate(members):
            if cluster_points:
                new_center = normalize(compute_mean(cluster_points))
            else:
                new_center = cluster_centers[cluster_idx]  # 空簇保留原中心
            new_centers.append(new_center)

        # ---------- 5. 收敛判断：中心不再变化就提前结束 ----------
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
    # 构造"按方向分组"的二维数据：三组点方向分别集中在 0°、120°、240°，模长完全随机。
    # 欧氏距离下同组点可能离得很远，但方向一致——正好体现余弦相似度聚类的价值
    rng = random.Random(42)

    def directional_points(center_angle_deg, spread_deg, count):
        """生成 count 个方向在 center_angle_deg 附近、模长随机的点"""
        points = []
        for _ in range(count):
            angle = math.radians(center_angle_deg + rng.gauss(0, spread_deg))
            radius = rng.uniform(0.5, 5.0)
            points.append([radius * math.cos(angle), radius * math.sin(angle)])
        return points

    data_points = (
        directional_points(0, 8, 100)
        + directional_points(120, 8, 100)
        + directional_points(240, 8, 100)
    )

    cluster_centers, cluster_labels = spherical_k_means(data_points, num_clusters=3)

    print("各簇中心方向（单位向量）:")
    for center in cluster_centers:
        rounded_center = []
        for value in center:
            rounded_center.append(round(value, 3))
        print(rounded_center)
    cluster_sizes = []
    for cluster_idx in range(3):
        cluster_sizes.append(cluster_labels.count(cluster_idx))
    print("各簇样本数:", cluster_sizes)
