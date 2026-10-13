# -*- coding: utf-8 -*-
"""Mini-Batch K-Means：每轮只用一小批随机样本更新中心（而不是全体样本），
牺牲一点精度换来大数据集上的大幅提速，网页分析、大规模特征聚类中的常用做法"""
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方（不开方也能比较远近）"""
    return sum((a - b) ** 2 for a, b in zip(point_a, point_b))


def mini_batch_k_means(data_points, num_clusters, batch_size=32, num_batches=100, seed=0):
    """
    Mini-Batch K-Means 主函数

    参数:
        data_points: 样本列表，每个样本是一个向量（如 [x, y]）
        num_clusters: 聚类中心个数
        batch_size: 每批参与更新的样本个数（远小于总样本数）
        num_batches: 抽多少批（相当于迭代多少轮）
        seed: 随机种子，保证结果可复现
    返回:
        cluster_centers: 最终聚类中心（num_clusters 个向量）
        cluster_labels:  每个样本所属簇的编号（与 data_points 等长的列表）
    """
    rng = random.Random(seed)

    # ---------- 1. 初始化：随机选 num_clusters 个样本作为初始中心 ----------
    cluster_centers = [list(point) for point in rng.sample(data_points, num_clusters)]
    update_counts = [0] * num_clusters  # 各中心被更新过的次数，用来算学习率

    # ---------- 2. 小批量更新：每批只抽 batch_size 个样本，逐个微调最近的中心 ----------
    for _ in range(num_batches):
        batch = rng.choices(data_points, k=batch_size)  # 有放回地抽一小批

        for point in batch:
            # 找当前样本最近的中心
            nearest_idx = min(
                range(num_clusters),
                key=lambda idx: squared_distance(point, cluster_centers[idx]),
            )
            # 更新次数越多学习率越小：中心向当前样本滑动的步长逐步衰减
            update_counts[nearest_idx] += 1
            learning_rate = 1.0 / update_counts[nearest_idx]
            cluster_centers[nearest_idx] = [
                (1 - learning_rate) * c + learning_rate * p
                for c, p in zip(cluster_centers[nearest_idx], point)
            ]

    # ---------- 3. 用最终中心给全体样本分配簇编号 ----------
    cluster_labels = [
        min(range(num_clusters), key=lambda idx: squared_distance(point, cluster_centers[idx]))
        for point in data_points
    ]

    return cluster_centers, cluster_labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点（与 k_means.py 相同）
    rng = random.Random(42)
    data_points = (
        [[rng.gauss(0, 0.5), rng.gauss(0, 0.5)] for _ in range(100)]      # 中心 (0,0)
        + [[rng.gauss(5, 0.5), rng.gauss(5, 0.5)] for _ in range(100)]    # 中心 (5,5)
        + [[rng.gauss(10, 0.5), rng.gauss(10, 0.5)] for _ in range(100)]  # 中心 (10,0)
    )

    cluster_centers, cluster_labels = mini_batch_k_means(data_points, num_clusters=3)

    print("聚类中心:")
    for center in cluster_centers:
        print([round(value, 3) for value in center])
    print("各簇样本数:", [cluster_labels.count(j) for j in range(3)])
