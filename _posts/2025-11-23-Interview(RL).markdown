---
layout: post
title:  "Interview(MPC)"
subtitle: "RL相关"
date:   2025-11-23 21:48:00
categories: [jotting, review]
---

# RL的本质是什么？

### 贝尔曼最优公式-A special Bellman equation
# PPO算法完整理解链路（从策略迭代到实现）

## 1. 从强化学习目标出发

强化学习的目标是最大化期望回报：

$$
J(\theta) = \mathbb{E}_{\tau \sim \pi_\theta}[R(\tau)]
$$

其中：

* $\pi_\theta$：策略
* $\tau$：轨迹
* $R(\tau)$：轨迹回报

关键困难在于：

期望是对“依赖参数 $\theta$ 的分布”取的

因此不能直接对期望内部求导。

---

## 2. Policy Gradient 的核心来源（log π 的由来）

为了计算：

$$
\nabla_\theta J(\theta)
$$

使用关键恒等式（log trick）：

$$
\nabla_\theta p(x) = p(x)\nabla_\theta \log p(x)
$$

应用后得到：

$$
\nabla J(\theta)
=

\mathbb{E}_{\tau \sim \pi_\theta}
\left[
R(\tau)\nabla \log P(\tau|\theta)
\right]
$$

轨迹概率：

$$
P(\tau)
=

\prod_t \pi_\theta(a_t|s_t) P(s_{t+1}|s_t,a_t)
$$

由于环境转移与 $\theta$ 无关：

$$
\nabla \log P(\tau)
=
\sum_t \nabla \log \pi_\theta(a_t|s_t)
$$

最终得到：

$$
\nabla J(\theta)
=
\mathbb{E}
\left[
\sum_t \nabla \log \pi_\theta(a_t|s_t);R(\tau)
\right]
$$

进一步优化为：

$$
\nabla J(\theta)
=
\mathbb{E}
\left[
\nabla \log \pi_\theta(a_t|s_t);A_t
\right]
$$

---

### 2.1 关键理解

* $\log \pi$ 不是人为引入的
* 而是为了把“分布的梯度”变成“可采样的期望”

直觉解释：

* $\nabla \log \pi(a|s)$：如何改变参数会让该动作概率变大
* $A_t$：这个方向值不值得走


### 2.2 Advantage (A_t) 的由来与意义

Advantage 定义为：

$$
A^\pi(s,a) = Q^\pi(s,a) - V^\pi(s)
$$

其中，$Q^\pi(s,a)$ 表示在状态 $s$ 执行动作 $a$ 后的期望回报，$V^\pi(s)$ 表示在该状态下按照当前策略的平均回报。因此：

$A(s,a)$ 表示“该动作相对于当前策略平均水平好多少”。

这一定义的核心作用是提供一个**相对评价基准**：

* 若 $A > 0$，说明该动作优于平均，应提高其概率
* 若 $A < 0$，说明该动作劣于平均，应降低其概率

从计算上，$A_t$ 通常无法直接获得，因此通过估计得到，例如：

* Monte Carlo：$A_t \approx R_t - V(s_t)$
* TD误差：$\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$
* GAE：对 TD 误差的加权累积

其中 TD 误差本身表示“实际结果相对于预期的偏差”，因此 Advantage 本质上是在时间维度上传播这种“好/坏信号”。

最终，Advantage 可以看作是：

**策略改进的信号（policy improvement signal），决定每个采样动作应该被强化还是被抑制。**

---

## 3. 策略迭代视角（PPO的理论归属）

策略迭代包含两步：

1. Policy Evaluation（估计价值函数）
2. Policy Improvement（改进策略）

PPO对应：

* Critic：估计 $V(s)$ 或 $A(s,a)$
* Actor：更新 $\pi_\theta$

因此：

PPO属于 Approximate Policy Iteration

---

## 4. 为什么不能直接做策略改进？

理想策略改进：

$$
\pi_{new}(s) = \arg\max_a Q^\pi(s,a)
$$

但在神经网络中：

* 一步更新可能过大
* 数据来自旧策略
* 分布发生变化

导致：

优化目标失效（distribution mismatch）

---

## 5. 关键问题：旧数据优化新策略

采样来自：

$$
\pi_{old}
$$

但目标是：

$$
\pi_\theta
$$

不能直接写：

$$
\mathbb{E}_{\pi_\theta}[...]
$$

---

## 6. Importance Sampling（ratio的来源）

核心公式：

$$
\mathbb{E}_{x \sim \pi_\theta}[f(x)]
=

\mathbb{E}_{x \sim \pi _{old}}
\left[
\frac{\pi_\theta(x)}{\pi_{old}(x)} f(x)
\right]
$$

应用到策略：

$$
r_t(\theta)
=

\frac{\pi_\theta(a_t|s_t)}
{\pi_{old}(a_t|s_t)}
$$

---

### 关键理解：ratio的含义

这个比值不是“两个策略函数相除”，而是：

在同一个 $(s,a)$ 上概率的比值

它表示：

新策略对该动作的“偏好变化程度”

例如：

* $r_t > 1$：更偏好该动作
* $r_t < 1$：抑制该动作

---

### 更本质的解释

$r_t$ 是一个 importance weight（重要性权重）

作用是：

用旧策略数据，估计新策略的期望

---

## 7. TRPO：引入 Trust Region

优化目标：

$$
\max_\theta
\mathbb{E}
\left[
r_t(\theta) A_t
\right]
$$

约束：

$$
D_{KL}(\pi_{old} | \pi_\theta) \le \delta
$$

---

### 核心思想

在“策略变化不大”的前提下进行改进

原因：

* advantage 是基于旧策略估计的
* 变化太大会导致估计失效

---

### 本质解释

TRPO做的是：

constrained policy improvement

即：

在 trust region 内寻找最优策略

---

## 8. PPO：TRPO的工程近似

PPO用 clip 替代 KL 约束：

$$
r_t(\theta) = \frac{\pi_\theta}{\pi_{old}}
$$

限制：

$$
r_t \in [1-\epsilon, 1+\epsilon]
$$

目标函数：

$$
L^{CLIP}
=

\mathbb{E}
\left[
\min\left(
r_t A_t,;
\text{clip}(r_t,1-\epsilon,1+\epsilon)A_t
\right)
\right]
$$

---

### clip的作用

限制：

概率变化不能过大

避免：

* 策略崩溃
* KL爆炸
* 过拟合 advantage

---

## 9. PPO整体工作流程

1. 使用 $\pi_{old}$ 与环境交互，采样数据
2. 计算 $A_t$（通常用 GAE）
3. 计算 $r_t$
4. 优化 PPO 目标函数
5. 更新 $\theta$
6. 重复

---

## 10. 全部核心逻辑总结

PPO可以统一理解为：

在近似策略迭代框架下，通过 importance sampling 修正分布，并使用 trust region 限制策略更新，从而实现稳定的策略优化

---

## 11. 最关键的三点理解

### 1. 为什么有 log π

* 为了把分布梯度转成期望
* 使 Monte Carlo 估计成为可能

---

### 2. 为什么有 ratio

* 用旧策略数据估计新策略
* 本质是 importance sampling

---

### 3. 为什么要 clip / trust region

* 防止策略更新过大
* 保证 advantage 仍然有效

---

## 12. 一句话总括

PPO = Policy Gradient + Importance Sampling + Trust Region + Approximate Policy Iteration
