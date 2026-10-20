# -*- coding: utf-8 -*-
"""模糊 C-Means（Fuzzy C-Means, FCM）：K-Means 的"软聚类"版本。
样本不再非此即彼地属于某一个簇，而是以 0~1 的隶属度同时属于所有簇（每个样本的隶属度之和为 1），
适合类别边界模糊的场景，典型应用是医学影像分割"""
import random


def squared_distance(point_a, point_b):
    """两点之间欧氏距离的平方（不开方也能比较远近）"""
    distance = 0.0
    for coordinate_a, coordinate_b in zip(point_a, point_b):
        difference = coordinate_a - coordinate_b
        distance += difference * difference
    return distance


def fuzzy_c_means(data_points, num_clusters, fuzziness=2.0, max_iters=100,
                  tolerance=1e-6, seed=0):
    """
    模糊 C-Means 主函数

    参数:
        data_points: 样本列表，每个样本是一个向量（如 [x, y]）
        num_clusters: 簇个数
        fuzziness: 模糊系数 m（必须 >1），越大聚类越"模糊"，通常取 2
        max_iters: 最大迭代次数（防止不收敛时死循环）
        tolerance: 中心变化量小于该值时认为收敛
        seed: 随机种子，保证结果可复现
    返回:
        cluster_centers: 最终聚类中心（num_clusters 个向量）
        memberships: 隶属度矩阵，memberships[i][j] 为样本 i 属于簇 j 的程度
    """
    rng = random.Random(seed)
    num_points = len(data_points)
    num_dims = len(data_points[0])
    exponent = 2.0 / (fuzziness - 1)  # 隶属度更新公式中的指数

    # ---------- 1. 初始化：每个样本一组随机的、总和为 1 的隶属度 ----------
    memberships = []
    for _ in range(num_points):
        raw_values = []
        for _ in range(num_clusters):
            raw_values.append(rng.random())
        total = 0.0
        for value in raw_values:
            total += value

        normalized_values = []
        for value in raw_values:
            normalized_values.append(value / total)
        memberships.append(normalized_values)

    cluster_centers = None
    for _ in range(max_iters):
        # ---------- 2. 更新中心：以"隶属度的 m 次方"为权重的加权平均 ----------
        new_centers = []
        for cluster_idx in range(num_clusters):
            weights = []
            for point_idx in range(num_points):
                weight = memberships[point_idx][cluster_idx] ** fuzziness
                weights.append(weight)

            weight_sum = 0.0
            for weight in weights:
                weight_sum += weight

            center = []
            for dimension_idx in range(num_dims):
                weighted_sum = 0.0
                for point_idx, weight in enumerate(weights):
                    weighted_sum += weight * data_points[point_idx][dimension_idx]
                center.append(weighted_sum / weight_sum)
            new_centers.append(center)

        # ---------- 3. 更新隶属度：离哪个中心越近，属于它的程度越高 ----------
        for point_idx, point in enumerate(data_points):
            distances = []
            for center in new_centers:
                distances.append(squared_distance(point, center))

            min_distance = distances[0]
            for distance in distances[1:]:
                if distance < min_distance:
                    min_distance = distance

            if min_distance == 0:  # 样本恰好落在某中心上：直接完全属于该簇
                new_memberships = []
                for distance in distances:
                    if distance == 0:
                        new_membership = 1.0
                    else:
                        new_membership = 0.0
                    new_memberships.append(new_membership)
                memberships[point_idx] = new_memberships
            else:
                new_memberships = []
                for cluster_idx in range(num_clusters):
                    denominator = 0.0
                    for other_idx in range(num_clusters):
                        denominator += (
                            distances[cluster_idx] / distances[other_idx]
                        ) ** exponent
                    new_memberships.append(1.0 / denominator)
                memberships[point_idx] = new_memberships

        # ---------- 4. 收敛判断：中心几乎不再移动就提前结束 ----------
        if cluster_centers is not None:
            converged = True
            for old_center, new_center in zip(cluster_centers, new_centers):
                for old_coordinate, new_coordinate in zip(old_center, new_center):
                    if abs(old_coordinate - new_coordinate) >= tolerance:
                        converged = False

            if converged:
                break
        cluster_centers = new_centers

    return cluster_centers, memberships


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

    cluster_centers, memberships = fuzzy_c_means(data_points, num_clusters=3)

    print("聚类中心:")
    for center in cluster_centers:
        rounded_center = []
        for value in center:
            rounded_center.append(round(value, 3))
        print(rounded_center)
    print("前 3 个样本的隶属度（每行三列之和为 1）:")
    for row in memberships[:3]:
        rounded_row = []
        for value in row:
            rounded_row.append(round(value, 3))
        print(rounded_row)
    # 取隶属度最大的簇作为"硬"聚类结果，统计各簇样本数
    hard_labels = []
    for row in memberships:
        best_cluster_idx = 0
        highest_membership = row[0]
        for cluster_idx in range(1, 3):
            if row[cluster_idx] > highest_membership:
                best_cluster_idx = cluster_idx
                highest_membership = row[cluster_idx]
        hard_labels.append(best_cluster_idx)
    cluster_sizes = []
    for cluster_idx in range(3):
        cluster_sizes.append(hard_labels.count(cluster_idx))
    print("各簇样本数:", cluster_sizes)
