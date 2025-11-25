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

## 一、在第$k$次迭代中
1. 线性化 dynamics 和 constraints
2. 二次近似 cost
3. 得到一个 quadratic program (QP)
4. 求解 QP 得到一个 增量方向
5. 更新轨迹，重复迭代