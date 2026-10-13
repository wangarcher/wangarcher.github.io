# -*- coding: utf-8 -*-
"""K-Medoids（K 中心点）：思路与 K-Means 相同，但"中心"必须是真实样本点（medoid），
而不是均值点。对离群点更鲁棒（均值会被极端值拉偏，真实样本点不会），中心也更好解释"""
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方（不开方也能比较远近）"""
    return sum((a - b) ** 2 for a, b in zip(point_a, point_b))


def total_distance(candidate, cluster_points):
    """candidate 到簇内所有成员的距离平方总和，越小说明它越适合当这个簇的中心点"""
    return sum(squared_distance(candidate, point) for point in cluster_points)


def k_medoids(data_points, num_clusters, max_iters=100, seed=0):
    """
    K-Medoids 主函数

    参数:
        data_points: 样本列表，每个样本是一个向量（如 [x, y]）
        num_clusters: 中心点个数
        max_iters: 最大迭代次数（防止不收敛时死循环）
        seed: 随机种子，保证结果可复现
    返回:
        medoids: 最终中心点（num_clusters 个真实样本）
        cluster_labels: 每个样本所属簇的编号（与 data_points 等长的列表）
    """
    rng = random.Random(seed)

    # ---------- 1. 初始化：随机选 num_clusters 个真实样本作为中心点 ----------
    medoids = rng.sample(data_points, num_clusters)

    for _ in range(max_iters):
        # ---------- 2. 分配步骤：每个样本归入最近的中心点 ----------
        cluster_labels = [
            min(range(num_clusters), key=lambda idx: squared_distance(point, medoids[idx]))
            for point in data_points
        ]

        # ---------- 3. 更新步骤：在每个簇里挑"到其他成员总距离最小"的样本当新中心点 ----------
        members = [[] for _ in range(num_clusters)]  # members[j] 为第 j 簇的全部样本
        for point, label in zip(data_points, cluster_labels):
            members[label].append(point)
        new_medoids = [
            min(cluster, key=lambda candidate: total_distance(candidate, cluster))
            if cluster else medoids[j]  # 空簇保留原中心点
            for j, cluster in enumerate(members)
        ]

        # ---------- 4. 收敛判断：中心点不再变化就提前结束 ----------
        if new_medoids == medoids:
            break
        medoids = new_medoids

    return medoids, cluster_labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点（与 k_means.py 相同）
    rng = random.Random(42)
    data_points = (
        [[rng.gauss(0, 0.5), rng.gauss(0, 0.5)] for _ in range(100)]      # 中心 (0,0)
        + [[rng.gauss(5, 0.5), rng.gauss(5, 0.5)] for _ in range(100)]    # 中心 (5,5)
        + [[rng.gauss(10, 0.5), rng.gauss(10, 0.5)] for _ in range(100)]  # 中心 (10,0)
    )

    medoids, cluster_labels = k_medoids(data_points, num_clusters=3)

    print("中心点（medoid，是真实样本）:")
    for medoid in medoids:
        print([round(value, 3) for value in medoid])
    print("各簇样本数:", [cluster_labels.count(j) for j in range(3)])
