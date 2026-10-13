# -*- coding: utf-8 -*-
"""Kernel K-Means（核 K-Means）：把样本经核函数映射到高维特征空间后再做 K-Means，
借助"核技巧"不显式计算映射，能分开普通 K-Means 搞不定的非凸簇（如同心圆环）。
这里用最常用的高斯核（RBF 核）"""
import math
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方"""
    return sum((a - b) ** 2 for a, b in zip(point_a, point_b))


def rbf_kernel(point_a, point_b, gamma):
    """高斯核 k(a,b) = exp(-gamma * ||a-b||^2)：值越接近 1 表示两点越相似"""
    return math.exp(-gamma * squared_distance(point_a, point_b))


def kernel_k_means(data_points, num_clusters, gamma=1.0, max_iters=100, seed=0):
    """
    Kernel K-Means 主函数

    参数:
        data_points: 样本列表，每个样本是一个向量（如 [x, y]）
        num_clusters: 簇个数
        gamma: RBF 核参数，越大越"挑剔"（只有非常近的点才算相似）
        max_iters: 最大迭代次数（防止不收敛时死循环）
        seed: 随机种子，保证结果可复现
    返回:
        cluster_labels: 每个样本所属簇的编号（与 data_points 等长的列表）

    原理：样本到"簇中心"（特征空间中的均值）的距离可以完全用核矩阵 K 表达，
    不需要知道映射 φ 本身：
        ||φ(x) - μ_j||² = K_ii - (2/|C_j|)·Σ_{l∈C_j} K_il + (1/|C_j|²)·Σ_{l,m∈C_j} K_lm
    """
    rng = random.Random(seed)
    num_points = len(data_points)

    # ---------- 1. 预计算核矩阵（对称矩阵，之后所有迭代都只查表） ----------
    kernel_matrix = [
        [rbf_kernel(point_a, point_b, gamma) for point_b in data_points]
        for point_a in data_points
    ]

    # ---------- 2. 初始化：随机给每个样本分配一个簇编号 ----------
    cluster_labels = [rng.randrange(num_clusters) for _ in range(num_points)]

    for _ in range(max_iters):
        # ---------- 3. 分配步骤：用核矩阵算特征空间距离，取最近的簇 ----------
        new_labels = []
        for i in range(num_points):
            best_label, best_dist = None, None
            for j in range(num_clusters):
                member_indices = [l for l in range(num_points) if cluster_labels[l] == j]
                if not member_indices:
                    continue  # 空簇没有中心，跳过

                size = len(member_indices)
                dist = (
                    kernel_matrix[i][i]  # ||φ(x)||²
                    - 2.0 / size * sum(kernel_matrix[i][l] for l in member_indices)
                    + 1.0 / size ** 2 * sum(
                        kernel_matrix[l][m]
                        for l in member_indices for m in member_indices
                    )
                )
                if best_dist is None or dist < best_dist:
                    best_label, best_dist = j, dist
            new_labels.append(best_label)

        # ---------- 4. 收敛判断：分配不再变化就提前结束 ----------
        if new_labels == cluster_labels:
            break
        cluster_labels = new_labels

    return cluster_labels


if __name__ == "__main__":
    # 构造非凸数据：内外两个同心圆环。普通 K-Means 必然把圆环"切月饼"式分错，
    # 核 K-Means 在特征空间中能把两个环整体分开
    rng = random.Random(0)

    def ring_points(radius, count, noise):
        """以原点为圆心、半径 radius 的圆环附近随机取 count 个点"""
        points = []
        for _ in range(count):
            angle = rng.uniform(0, 2 * math.pi)
            r = radius + rng.gauss(0, noise)
            points.append([r * math.cos(angle), r * math.sin(angle)])
        return points

    data_points = ring_points(radius=1.0, count=100, noise=0.06)   # 内环
    data_points += ring_points(radius=3.0, count=100, noise=0.06)  # 外环

    cluster_labels = kernel_k_means(data_points, num_clusters=2, gamma=0.5)

    print("各簇样本数:", [cluster_labels.count(j) for j in range(2)])
    print("内环被分到各簇的数量:", [cluster_labels[:100].count(j) for j in range(2)])
    print("外环被分到各簇的数量:", [cluster_labels[100:].count(j) for j in range(2)])
