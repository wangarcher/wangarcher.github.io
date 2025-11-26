---
layout: post
title:  "Interview(MPC)"
subtitle: "MPC相关"
date:   2025-11-23 21:48:00
categories: [jotting, review]
---

# MPC的本质是什么？

# MPC的典型优化过程是什么？

# 基于OSQP求解器的MPC优化过程是什么？
## 一、MPC问题到QP问题的转换
**Model Predictive Control的典型形式：**

$$
\begin{aligned}
\min_{x_k,u_k}\quad & (x_N-x_r)^\top Q_N (x_N-x_r) + \sum_{k=0}^{N-1} (x_k-x_r)^\top Q (x_k-x_r) + u_k^\top R u_k \\
\text{s.t.}\quad & x_{k+1} = A x_k + B u_k, \\
& x_{\min} \le x_k \le x_{\max}, \\
& u_{\min} \le u_k \le u_{\max}, \\
& x_0 = \bar{x}.
\end{aligned}
$$

**标准QP形式：**

$$
\begin{aligned}
\text{minimize}\quad & \tfrac{1}{2} z^\top P z + q^\top z \\
\text{subject to}\quad & l \le A_c\, z \le u
\end{aligned}
$$

其中，策变量按时间堆叠为$z = [x_0^\top,x_1^\top,\dots,x_N^\top,u_0^\top,\dots,u_{N-1}^\top]^\top$。

**Hessian：$P$**

$$
P = \mathrm{diag}(Q,\,Q,\,\dots,\,Q_N,\,R,\,\dots,\,R)
$$

**Gradient：$q$**

$$
q = \begin{bmatrix}
- Q x_r \\
- Q x_r \\
\vdots \\
- Q_N x_r \\
0 \\
\vdots \\
0
\end{bmatrix}
$$

前面$N+1$个块对应状态项的线性项（一次项，也就是MPC代价函数乘开后获得的值），后面对应控制输入的零项。

**线性约束矩阵：$A_c$**

按示例构造的稀疏块矩阵（等式动力学 + 状态/输入的直接投影）：

$$
A_c =
\left[
\begin{array}{ccccc|cccc}
 -I & 0 & 0 & \cdots & 0 & 0 & 0 & \cdots & 0\\
 A & -I & 0 & \cdots & 0 & B & 0 & \cdots & 0\\
 0 & A & -I & \cdots & 0 & 0 & B & \cdots & 0\\
 \vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots\\
 0 & 0 & 0 & \cdots & -I & 0 & 0 & \cdots & B\\
 \hline
 I & 0 & 0 & \cdots & 0 & 0 & 0 & \cdots & 0\\
 0 & I & 0 & \cdots & 0 & 0 & 0 & \cdots & 0\\
 0 & 0 & I & \cdots & 0 & 0 & 0 & \cdots & 0\\
 \vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots\\
 0 & 0 & 0 & \cdots & I & 0 & 0 & \cdots & 0\\
 0 & 0 & 0 & \cdots & 0 & I & 0 & \cdots & 0\\
 \vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots
\end{array}
\right]
$$

- 上半部分（直到分隔线）表示动力学等式 $x_{k+1} - A x_k - B u_k = 0$（用 $-I,A$ 与 $B$ 的块）。
- 下半部分为把状态与输入直接投影到不等式约束（即把 $x_i$ 和 $u_i$ 拷贝为独立行以便施加上下界）。

**下界与上界：$l,u$**

示例中将等式约束（初始条件）与不等式约束（状态/输入上下界）合并为：

$$
\begin{aligned}
l &= \begin{bmatrix}
- x_0 \\
0 \\
\vdots \\
0 \\
x_{\min} \\
\vdots \\
x_{\min} \\
u_{\min} \\
\vdots \\
u_{\min}
\end{bmatrix}, \qquad
u = \begin{bmatrix}
- x_0 \\
0 \\
\vdots \\
0 \\
x_{\max} \\
\vdots \\
x_{\max} \\
u_{\max} \\
\vdots \\
u_{\max}
\end{bmatrix}
\end{aligned}
$$

## 二、QP问题在OSQP中的求解。

**x-update**

在 ADMM 的每次迭代中，OSQP的x-update要求解下面的最小化问题：

$$
x^{k+1}
= \arg\min_x 
\left(
\frac{1}{2} x^\top H x + g^\top x
+ \frac{\rho}{2}\|Ax - (z^k - u^k)\|^2
\right)
$$

这是一个标准的二次优化问题。对其求导并令梯度为零即可得到线性系统：

$$
(H + \rho A^\top A)x^{k+1}
= -g + \rho A^\top (z^k - u^k)
$$

因此**x-update 的本质就是解一个线性方程**：

$$
Mx = q,
\qquad
M = H + \rho A^\top A,\ 
q = -g + \rho A^\top (z^k - u^k)
$$

为了加速求解，OSQP 会在初始化时对矩阵 $M$（或更大的 KKT 系统）执行一次稀疏的**LDLᵀ（KKT）分解**。  
这样后续每次迭代只需进行快速的前代与回代即可求得 $x^{k+1}$，无需重新分解矩阵。



# OCS2中的SQP优化过程是什么？

## 零、在第$k$次迭代中
1. 线性化 dynamics 和 constraints
2. 二次近似 cost
3. 得到一个 quadratic program (QP)
4. 求解 QP 得到一个 增量方向
5. 更新轨迹，重复迭代

## 一、给定初始控制序列 $u_{0:N-1}$

通常来自：
- 上一次 MPC 控制循环的解，或  
- 简单的 warm start（零控制 / PD 控制等）。  

## 二、Forward Rollout —— 生成名义轨迹

基于系统动力学：
$$
\dot{x}(t) = f(x(t), u(t))
$$

从初始状态 $x_0$ 前向积分：

$$
x_{k+1} = x_k + \int_0^{\Delta t} f(x(t), u_k)\, dt
$$
```
注：OCS2中的积分实际上主要是直接加，RK4都没用，这可能是导致他性能不佳的原因。
```

得到名义轨迹：
$$
\{x_k, u_k\}_{k=0}^{N}
$$

此轨迹满足动力学，用于后续线性化。

## 三、对动力学与成本函数进行二阶泰勒展开（Local Bellman Expansion）

### 动力学线性化：
$$
\delta x_{k+1} \approx A_k \delta x_k + B_k \delta u_k
$$

其中：
$$
A_k = \frac{\partial f}{\partial x}\Big|_{x_k,u_k}, \qquad 
B_k = \frac{\partial f}{\partial u}\Big|_{x_k,u_k}
$$

### Cost 二阶展开：
$$
\ell(x_k,u_k) \approx \ell_k 
+ q_k^\top \delta x_k + r_k^\top \delta u_k 
+ \frac12 
\begin{bmatrix} \delta x_k \\ \delta u_k \end{bmatrix}^\top
\begin{bmatrix} Q_k & P_k \\ P_k^\top & R_k \end{bmatrix}
\begin{bmatrix} \delta x_k \\ \delta u_k \end{bmatrix}
$$

终端成本同理展开。

## 四、构造等效的 Quadratic Program (QP)

目标为：
$$
\min_{\delta x,\delta u}
\sum_{k=0}^{N-1} \left(
\frac12 
\begin{bmatrix}\delta x_k \\ \delta u_k\end{bmatrix}^\top
H_k
\begin{bmatrix}\delta x_k \\ \delta u_k\end{bmatrix}
+
g_k^\top
\begin{bmatrix}\delta x_k \\ \delta u_k\end{bmatrix}
\right)
+
\frac12 \delta x_N^\top Q_N \delta x_N + q_N^\top\delta x_N
$$

约束为线性化动力学：
$$
\delta x_{k+1} = A_k \delta x_k + B_k \delta u_k
$$

---

## 五、用 Riccati-like backward sweep 求解 QP（也称 LQ Optimal Control）

利用动态规划，做一次 backward pass：

### value function：
$$
V_k(\delta x_k) = 
\frac12 \delta x_k^\top S_k \delta x_k + s_k^\top \delta x_k
$$

### Riccati 递推：
$$
K_k = -(R_k + B_k^\top S_{k+1} B_k)^{-1}(P_k^\top + B_k^\top S_{k+1} A_k)
$$

$$
d_k = -(R_k + B_k^\top S_{k+1} B_k)^{-1}(r_k + B_k^\top s_{k+1})
$$

更新：
$$
S_k = Q_k + A_k^\top S_{k+1} A_k + 
K_k^\top(R_k + B_k^\top S_{k+1}B_k)K_k
$$

$$
s_k = q_k + A_k^\top s_{k+1} +
K_k^\top (R_k + B_k^\top S_{k+1}B_k)d_k
$$

最终得到控制修正：
$$
\delta u_k = K_k\, \delta x_k + d_k
$$

---

## 六、更新控制并执行 line search

更新控制：
$$
u_k^{new} = u_k + \alpha \, \delta u_k
$$

其中 $\alpha \in (0,1]$ 由 line search 确定，以保证成本下降。

---

## 七、重复迭代直到收敛

判断条件：
- 控制更新幅度变小  
- 成本下降足够  
- 达到最大迭代次数  

然后输出：
- 最优控制序列  
- 最优状态轨迹（通过最后一次 rollout 得到）



# SQP中Riccati backward/forward pass 的具体步骤

## A. 目标（Purpose）
在 SQP/ILQR 中，当我们把原非线性问题在第 $k$ 次迭代近似为线性动力学 + 二次代价（LQ subproblem）时，可以通过 Riccati backward recursion 求出局部最优控制增量：
$$
\delta u_i = k_i + K_i\,\delta x_i
$$
其中：
- $k_i$：开环控制增量  
- $K_i$：反馈增益  

该策略确保 LQ 子问题的 **精确最优解**，并具有线性复杂度 $O(N)$（对 horizon $N$）。

---

## B. 局部增量代价定义（Quadratic approximation）
定义状态与控制的增量：
$$
\delta x_i = x_i - x_{k,i}, \quad \delta u_i = u_i - u_{k,i}.
$$

二次近似的阶段代价写为：
$$
\ell_i(\delta x_i,\delta u_i) \approx 
\frac{1}{2}
\begin{bmatrix}
\delta x \\ \delta u
\end{bmatrix}^\top
\begin{bmatrix}
Q_{xx} & Q_{xu} \\
Q_{ux} & Q_{uu}
\end{bmatrix}
\begin{bmatrix}
\delta x \\ \delta u
\end{bmatrix}
+
\begin{bmatrix}
q_x \\ q_u
\end{bmatrix}^\top
\begin{bmatrix}
\delta x \\ \delta u
\end{bmatrix}.
$$

动力学线性化：
$$
\delta x_{i+1} = A_i\,\delta x_i + B_i\,\delta u_i + d_i.
$$

最终代价：
$$
V_N(\delta x_N)=
\frac12\, \delta x_N^\top V_{xx,N}\,\delta x_N + V_{x,N}^\top \delta x_N.
$$

---

## C. Q-function（Bellman 局部展开）
定义：
$$
Q_i(\delta x_i,\delta u_i)
=
\ell_i(\delta x_i,\delta u_i)
+ 
V_{i+1}\left(A_i\delta x_i + B_i\delta u_i + d_i \right).
$$

代入值函数二次形式，可得：
$$
Q(\delta x,\delta u)
=
\frac12
\begin{bmatrix}
\delta x\\ \delta u
\end{bmatrix}^\top
\begin{bmatrix}
Q_{xx}^Q & Q_{xu}^Q \\
Q_{ux}^Q & Q_{uu}^Q
\end{bmatrix}
\begin{bmatrix}
\delta x\\ \delta u
\end{bmatrix}
+
\begin{bmatrix}
Q_x^Q\\ Q_u^Q
\end{bmatrix}^\top
\begin{bmatrix}
\delta x\\ \delta u
\end{bmatrix}
+ \text{const}.
$$

其中（常见计算公式）：
$$
\begin{aligned}
Q_{xx}^Q &= Q_{xx} + A_i^\top V_{xx,i+1} A_i, \\
Q_{uu}^Q &= Q_{uu} + B_i^\top V_{xx,i+1} B_i, \\
Q_{ux}^Q &= Q_{ux} + B_i^\top V_{xx,i+1} A_i, \\
Q_x^Q &= q_x + A_i^\top (V_{xx,i+1} d_i + V_{x,i+1}), \\
Q_u^Q &= q_u + B_i^\top (V_{xx,i+1} d_i + V_{x,i+1}).
\end{aligned}
$$

---

## D. 求解局部最优控制增量（optimal $\delta u$）
对 $Q$ 关于 $\delta u$ 求导并置零：
$$
\frac{\partial Q}{\partial \delta u}
=
Q_{uu}^Q\,\delta u + Q_{ux}^Q\,\delta x + Q_u^Q = 0.
$$

若 $Q_{uu}^Q$ 正定，可以得到：
$$
\delta u = k_i + K_i\delta x,
$$
其中：
$$
k_i = - (Q_{uu}^Q)^{-1} Q_u^Q,\qquad
K_i = - (Q_{uu}^Q)^{-1} Q_{ux}^Q.
$$

> 数值上必须保证 $Q_{uu}^Q$ 正定，否则要做正则化  
> $Q_{uu}^Q \leftarrow Q_{uu}^Q + \lambda I$（Levenberg–Marquardt 样式）。

---

## E. 值函数更新（Value function recursion）
将最优 $\delta u$ 代入 $Q$，得到新的时刻值函数：
$$
V_{xx,i} = Q_{xx}^Q + Q_{xu}^Q K_i,
$$
$$
V_{x,i} = Q_x^Q + Q_{xu}^Q k_i.
$$

这完成了一个时间步的 **backward** 递推。

Backward pass 从 $i=N-1$ 递推到 $i=0$：

```pseudo
# backward pass
V_x  = V_x_terminal
V_xx = V_xx_terminal

for i = N-1 ... 0:
    # build Q terms
    Q_xx, Q_uu, Q_ux, Q_x, Q_u = compute_Q_terms(A_i, B_i, V_xx, V_x)

    # regularize for positive-definite
    Q_uu = Q_uu + lambda * I

    # compute gains
    k_i = - inv(Q_uu) * Q_u
    K_i = - inv(Q_uu) * Q_ux

    # update value function
    V_x  = Q_x  + Q_xu * k_i
    V_xx = Q_xx + Q_xu * K_i
```
## F. Forward pass（利用策略生成新轨迹）

向前滚动，用新的策略更新控制与状态：
```
x = x0
for i = 0 ... N-1:
    delta_x = x - x_nom[i]
    delta_u = k_i + K_i * delta_x
    u_new   = u_nom[i] + alpha * delta_u   # alpha: line search

    x = integrate_forward(x, u_new)        # ODE solver
```

Forward pass 的主要作用：

得到新的名义轨迹 

评估真实 cost / KKT violation 是否下降

为下一次迭代提供线性化基点