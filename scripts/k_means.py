# -*- coding: utf-8 -*-
"""最基础的 K-Means 聚类算法实现（纯 Python，无第三方依赖）"""
import random


def dist_sq(a, b):
    """两向量间的欧氏距离的平方（比较大小与距离本身等价，省去开方）"""
    return sum((x - y) ** 2 for x, y in zip(a, b))


def mean_vec(vectors):
    """一组向量的逐维均值"""
    n = len(vectors)
    return [sum(col) / n for col in zip(*vectors)]


def kmeans(X, k, max_iters=100, seed=0):
    """
    K-Means 主函数

    参数:
        X: 样本列表，每个样本是一个向量（如 [x, y]）
        k: 聚类中心个数
        max_iters: 最大迭代次数（防止不收敛时死循环）
        seed: 随机种子，保证结果可复现
    返回:
        centers: 最终聚类中心（k 个向量）
        labels:  每个样本所属簇的编号（与 X 等长的列表）
    """
    rng = random.Random(seed)

    # ---------- 1. 初始化：随机选 k 个样本作为初始中心 ----------
    centers = [list(x) for x in rng.sample(X, k)]  # 不放回抽取 k 个

    for _ in range(max_iters):
        # ---------- 2. 分配步骤：把每个样本分给最近的中心 ----------
        # labels[i] = 距样本 X[i] 最近的中心的编号
        labels = [
            min(range(k), key=lambda j: dist_sq(x, centers[j]))
            for x in X
        ]

        # ---------- 3. 更新步骤：把每个中心移动到其成员的平均位置 ----------
        groups = [[] for _ in range(k)]
        for x, label in zip(X, labels):
            groups[label].append(x)
        new_centers = [
            mean_vec(groups[j]) if groups[j] else centers[j]  # 空簇保留原中心，避免除零
            for j in range(k)
        ]

        # ---------- 4. 收敛判断：中心不再变化就提前结束 ----------
        if all(all(abs(a - b) < 1e-12 for a, b in zip(old, new))
               for old, new in zip(centers, new_centers)):
            break
        centers = new_centers

    return centers, labels


if __name__ == "__main__":
    # 构造一个简单的二维数据集：三团明显分开的点
    rng = random.Random(42)
    X = (
        [[rng.gauss(0, 0.5), rng.gauss(0, 0.5)] for _ in range(100)]    # 中心 (0,0)
        + [[rng.gauss(5, 0.5), rng.gauss(5, 0.5)] for _ in range(100)]  # 中心 (5,5)
        + [[rng.gauss(10, 0.5), rng.gauss(10, 0.5)] for _ in range(100)]  # 中心 (10,0)
    )

    centers, labels = kmeans(X, k=3)

    print("聚类中心:")
    for c in centers:
        print([round(v, 3) for v in c])
    print("各簇样本数:", [labels.count(j) for j in range(3)])
