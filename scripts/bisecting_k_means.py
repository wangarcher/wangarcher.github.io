# -*- coding: utf-8 -*-
"""二分 K-Means（Bisecting K-Means）：从"所有点算一个簇"出发，
每轮挑误差最大的簇用基础 K-Means(k=2) 一分为二，直到凑够目标簇数。
相比一次性划分，通常能得到更稳定、更接近全局最优的结果"""
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方（不开方也能比较远近）"""
    return sum((a - b) ** 2 for a, b in zip(point_a, point_b))


def compute_mean(points):
    """一组点的均值（逐维取平均）"""
    return [sum(each_dim) / len(points) for each_dim in zip(*points)]


def kmeans(data_points, num_clusters, max_iters=100, seed=0):
    """基础 K-Means（与 k_means.py 相同），供二分时调用"""
    rng = random.Random(seed)
    cluster_centers = [list(point) for point in rng.sample(data_points, num_clusters)]

    for _ in range(max_iters):
        cluster_labels = [
            min(range(num_clusters), key=lambda idx: squared_distance(point, cluster_centers[idx]))
            for point in data_points
        ]
        members = [[] for _ in range(num_clusters)]
        for point, label in zip(data_points, cluster_labels):
            members[label].append(point)
        new_centers = [
            compute_mean(members[j]) if members[j] else cluster_centers[j]
            for j in range(num_clusters)
        ]
        if all(all(abs(a - b) < 1e-12 for a, b in zip(old, new))
               for old, new in zip(cluster_centers, new_centers)):
            break
        cluster_centers = new_centers

    return cluster_centers, cluster_labels


def cluster_sse(points):
    """一个簇的误差平方和（SSE）：各成员到簇质心距离的平方之和，越大说明这个簇分得越差"""
    center = compute_mean(points)
    return sum(squared_distance(point, center) for point in points)


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
        worst_idx = max(
            range(len(clusters)),
            key=lambda idx: cluster_sse([data_points[i] for i in clusters[idx]]),
        )
        member_indices = clusters[worst_idx]

        # ---------- 2. 用基础 K-Means 把这个簇分成两半 ----------
        sub_points = [data_points[i] for i in member_indices]
        for _ in range(10):  # 极端情况（点重合）会分不出两半，换随机种子重试
            _, sub_labels = kmeans(sub_points, num_clusters=2, seed=rng.randint(0, 10**9))
            first_half = [i for i, label in zip(member_indices, sub_labels) if label == 0]
            second_half = [i for i, label in zip(member_indices, sub_labels) if label == 1]
            if first_half and second_half:
                break
        else:
            raise ValueError("簇内样本完全相同，无法继续二分")

        # ---------- 3. 用两个新簇替换被分裂的簇 ----------
        clusters[worst_idx:worst_idx + 1] = [first_half, second_half]

    # ---------- 4. 汇总：各簇质心 + 与输入顺序对齐的簇编号 ----------
    cluster_centers = [
        compute_mean([data_points[i] for i in member_indices]) for member_indices in clusters
    ]
    cluster_labels = [None] * len(data_points)
    for cluster_idx, member_indices in enumerate(clusters):
        for i in member_indices:
            cluster_labels[i] = cluster_idx

    return cluster_centers, cluster_labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点（与 k_means.py 相同）
    rng = random.Random(42)
    data_points = (
        [[rng.gauss(0, 0.5), rng.gauss(0, 0.5)] for _ in range(100)]      # 中心 (0,0)
        + [[rng.gauss(5, 0.5), rng.gauss(5, 0.5)] for _ in range(100)]    # 中心 (5,5)
        + [[rng.gauss(10, 0.5), rng.gauss(10, 0.5)] for _ in range(100)]  # 中心 (10,0)
    )

    cluster_centers, cluster_labels = bisecting_k_means(data_points, num_clusters=3)

    print("聚类中心:")
    for center in cluster_centers:
        print([round(value, 3) for value in center])
    print("各簇样本数:", [cluster_labels.count(j) for j in range(3)])
