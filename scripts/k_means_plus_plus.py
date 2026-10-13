# -*- coding: utf-8 -*-
"""K-Means++：只改进基础 K-Means 的"初始中心怎么挑"，之后的迭代完全相同。
初始中心彼此散开而不是扎堆，通常收敛更快、更不容易落入差的局部最优"""
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方（不开方也能比较远近）"""
    return sum((a - b) ** 2 for a, b in zip(point_a, point_b))


def compute_mean(points):
    """一组点的均值（逐维取平均）"""
    return [sum(each_dim) / len(points) for each_dim in zip(*points)]


def plus_plus_init(data_points, num_clusters, rng):
    """
    K-Means++ 初始化

    规则：第 1 个中心纯随机；之后每个新中心按 D(x)^2 加权抽取——
    离已有中心越远的点，被选为新中心的概率越大
    """
    centers = [list(rng.choice(data_points))]  # 第 1 个中心：纯随机

    while len(centers) < num_clusters:
        # 每个点到"离它最近的已有中心"的距离平方，作为被抽中的权重
        weights = [
            min(squared_distance(point, center) for center in centers)
            for point in data_points
        ]
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
        cluster_labels = [
            min(range(num_clusters), key=lambda idx: squared_distance(point, cluster_centers[idx]))
            for point in data_points
        ]

        # 更新步骤：把每个中心移动到其成员的平均位置
        members = [[] for _ in range(num_clusters)]  # members[j] 为第 j 簇的全部样本
        for point, label in zip(data_points, cluster_labels):
            members[label].append(point)
        new_centers = [
            compute_mean(members[j]) if members[j] else cluster_centers[j]  # 空簇保留原中心
            for j in range(num_clusters)
        ]

        # 收敛判断：中心不再变化就提前结束
        if all(all(abs(a - b) < 1e-12 for a, b in zip(old, new))
               for old, new in zip(cluster_centers, new_centers)):
            break
        cluster_centers = new_centers

    return cluster_centers, cluster_labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点（与 k_means.py 相同）
    rng = random.Random(42)
    data_points = (
        [[rng.gauss(0, 0.5), rng.gauss(0, 0.5)] for _ in range(100)]      # 中心 (0,0)
        + [[rng.gauss(5, 0.5), rng.gauss(5, 0.5)] for _ in range(100)]    # 中心 (5,5)
        + [[rng.gauss(10, 0.5), rng.gauss(10, 0.5)] for _ in range(100)]  # 中心 (10,0)
    )

    cluster_centers, cluster_labels = k_means_plus_plus(data_points, num_clusters=3)

    print("聚类中心:")
    for center in cluster_centers:
        print([round(value, 3) for value in center])
    print("各簇样本数:", [cluster_labels.count(j) for j in range(3)])
