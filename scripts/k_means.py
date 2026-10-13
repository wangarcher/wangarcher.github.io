# -*- coding: utf-8 -*-
"""最基础的 K-Means 聚类算法实现（纯 Python，无第三方依赖）"""
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方（不开方也能比较远近）"""
    return sum((a - b) ** 2 for a, b in zip(point_a, point_b))

def compute_mean(points):
    """一组点的均值（逐维取平均）"""
    return [sum(each_dim) / len(points) for each_dim in zip(*points)]

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
    cluster_centers = [list(point) for point in rng.sample(data_points, num_clusters)]

    for _ in range(max_iters):
        # ---------- 2. 分配步骤：把每个样本分给最近的中心 ----------
        cluster_labels = []  # cluster_labels[i] = 样本 data_points[i] 所属簇的编号
        for point in data_points:
            # 先假设 0 号中心最近，再把其余中心逐个比一遍
            nearest_idx = 0
            nearest_dist = squared_distance(point, cluster_centers[0])
            for idx in range(1, num_clusters):
                dist = squared_distance(point, cluster_centers[idx])
                if dist < nearest_dist:
                    nearest_idx = idx
                    nearest_dist = dist
            cluster_labels.append(nearest_idx)

        # ---------- 3. 更新步骤：把每个中心移动到其成员的平均位置 ----------
        members = [[] for _ in range(num_clusters)]  # members[j] 为第 j 簇的全部样本
        for point, label in zip(data_points, cluster_labels):
            members[label].append(point)
        new_centers = [
            compute_mean(members[j]) if members[j] else cluster_centers[j]  # 空簇保留原中心，避免除零
            for j in range(num_clusters)
        ]

        # ---------- 4. 收敛判断：中心不再变化就提前结束 ----------
        converged = True  # 先假设已收敛，发现任何一维还在动就推翻
        for old, new in zip(cluster_centers, new_centers):  # 新旧中心逐个配对
            for a, b in zip(old, new):  # 每个中心的坐标逐维配对
                if abs(a - b) >= 1e-12:  # 浮点误差容忍：小于该值当作没变
                    converged = False
        if converged:
            break
        cluster_centers = new_centers

    return cluster_centers, cluster_labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点
    rng = random.Random(42)
    data_points = (
        [[rng.gauss(0, 0.5), rng.gauss(0, 0.5)] for _ in range(100)]      # 中心 (0,0)
        + [[rng.gauss(5, 0.5), rng.gauss(5, 0.5)] for _ in range(100)]    # 中心 (5,5)
        + [[rng.gauss(10, 0.5), rng.gauss(10, 0.5)] for _ in range(100)]  # 中心 (10,0)
    )

    cluster_centers, cluster_labels = kmeans(data_points, num_clusters=3)

    print("聚类中心:")
    for center in cluster_centers:
        print([round(value, 3) for value in center])
    print("各簇样本数:", [cluster_labels.count(j) for j in range(3)])
