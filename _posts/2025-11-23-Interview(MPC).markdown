---
layout: post
title:  "Interview(MPC)"
subtitle: "MPC相关"
date:   2025-11-23 21:48:00
categories: [jotting, review]
---

# 先导：LQR、iLQR、NMPC 与 MPC 的关系

|   | LQR | 线性MPC (QP/OSQP) | iLQR | NMPC (SQP/OCS2) |
|---|---|---|---|---|
| 动力学 | 线性 $Ax+Bu$ | 线性 | 非线性 $f(x,u)$ | 非线性 |
| 代价 | 二次 | 二次 | 非线性（局部二次） | 非线性 |
| 约束 | 无 | 有 | 一般无硬约束 | 有 |
| 时域 | 无限 $\infty$ | 有限 $N$ | 有限 $N$ | 有限 $N$ |
| 求解 | 解析（Riccati） | 迭代（ADMM/QP） | 迭代（SQP + Riccati） | 迭代（SQP + Riccati） |

一句话：**LQR 是 MPC 去掉约束、去掉有限时域、去掉非线性后的解析极限；iLQR 是把 LQR 反复用在非线性系统线性化后的 LQ 子问题上；NMPC 则是线性 MPC 把动力学/代价/约束全部换成非线性。**

# LQR的本质是什么？

## 一、LQR的典型形式

文献中常写作无限时域的线性二次调节问题：

$$
\begin{aligned}
\min_{u_{0:\infty}}\quad & \sum_{k=0}^{\infty} \left( x_k^\top Q x_k + u_k^\top R u_k \right) \\
\text{s.t.}\quad & x_{k+1} = A x_k + B u_k,\\
& x_0 = \bar{x}.
\end{aligned}
$$

与本文档上面 MPC 形式的差别正是它的"思想"所在：

- **没有约束**：没有 $x_{\min}\le x_k\le x_{\max}$、$u_{\min}\le u_k\le u_{\max}$；
- **时域无限**：$N\to\infty$，因此终端代价项被隐式包含，不需要额外设计 $\ell_f$；
- **离线求解**：控制器只需在部署前算一次，在线只做一次矩阵乘法，不需要滚动优化。

## 二、LQR的解：从 Bellman 到代数 Riccati 方程

由于无约束、代价二次、动力学线性，值函数（cost-to-go）必然是二次型 $V(x)=x^\top S x$。代入 Bellman 方程：

$$
x^\top S x
= \min_u \left[ x^\top Q x + u^\top R u + (Ax+Bu)^\top S (Ax+Bu) \right]
$$

对 $u$ 求导并令梯度为零：

$$
2 R u + 2 B^\top S (Ax + Bu) = 0
\;\Longrightarrow\;
u^* = -\underbrace{(R+B^\top S B)^{-1} B^\top S A}_{K}\, x
$$

即 **LQR 的最优控制是一个静态线性状态反馈 $u^*=-Kx$**，把 $u^*$ 代回可得离散代数 Riccati 方程（DARE）：

$$
S = Q + A^\top S A - A^\top S B \left( R + B^\top S B \right)^{-1} B^\top S A
$$

当 $(A,B)$ 可稳、$(A,Q^{1/2})$ 可检测时，DARE 有唯一的半正定解 $S$，$K$ 随即确定且为常数增益。

## 二·补：DARE 的具体求解方法

$$
S = Q + A^\top S A - A^\top S B \left( R + B^\top S B \right)^{-1} B^\top S A
$$

上式对 $S$ 是**二次矩阵方程**（$S$ 与 $S^{-1}$ 同时出现），不能用"移项"解出 $S$；矩阵没有除法，方程的解 $S^*$ 是右端算子的**不动点**。求解方法有三类：

**方法一：Riccati 迭代（不动点迭代，实现最简单）**

取初值 $S^{(0)}=0$（或 $S^{(0)}=Q$）反复代入右端：

$$
S^{(t+1)} = Q + A^\top S^{(t)} A - A^\top S^{(t)} B \left( R + B^\top S^{(t)} B \right)^{-1} B^\top S^{(t)} A
$$

- $S^{(0)}=0$ 时序列**单调不减**、由下向上收敛；$S^{(0)}\succeq S^*$ 时单调不增、由上向下收敛；
- 局部渐近二次收敛，全局线性收敛；
- **本质**：这正是有限时域 Riccati 递推从终端 $S_N=0$ 反向跑到无穷远的极限 $\lim_{k\to-\infty}S_k$，即"无限时域 LQR = 有限时域递推迭代到收敛"。

```pseudo
S = 0                      # 或 Q
repeat:
    K = inv(R + B'*S*B) * (B'*S*A)
    S_new = Q + A'*S*A - A'*S*B*K
until norm(S_new - S) < eps
```

**方法二：辛矩阵特征分解 / Schur 法（一次分解得到解）**

令 $G = B R^{-1} B^\top$，构造离散时间辛矩阵：

$$
Z =
\begin{bmatrix}
A + G A^{-\top} Q & -G A^{-\top} \\
-A^{-\top} Q & A^{-\top}
\end{bmatrix}
$$

$Z$ 的特征值必然**成互反对出现**（$\lambda$ 与 $1/\lambda$ 配对），其中 $|\lambda|<1$ 的 $n$ 个恰好对应闭环极点。取这些特征向量拼成 $\begin{bmatrix}X_1\\X_2\end{bmatrix}$，则

$$
S^* = X_2 X_1^{-1}
$$

必须取**稳定**的那一半特征向量：取错的一半得到的 $S$ 虽满足方程，但闭环 $A-BK$ 反而不稳定。

$A$ 不可逆时改用**辛 pencil 的广义特征值分解（QZ）**：

$$
\begin{bmatrix} A & 0 \\ -Q & I \end{bmatrix}
-\lambda
\begin{bmatrix} I & G \\ 0 & A^\top \end{bmatrix}
$$

对 stable deflating subspace 做 ordered Schur 分解后同样取 $S^*=X_2X_1^{-1}$。MATLAB `dlqr`/`dare`、`scipy.linalg.solve_discrete_are` 走的就是这条路。

**方法三：结构保持倍增算法（SDA）**

利用 Riccati 方程自身的辛结构反复做"平方"，$O(\log(1/\epsilon))$ 次矩阵乘/求逆即可收敛，quadratic 收敛且 $A$ 奇异也能处理，数值上比方法二更稳。

| 方法 | 复杂度 | 特点 |
|---|---|---|
| Riccati 迭代 | $O(Tn^3)$ | 最简单，等价于"跑很长时域" |
| 辛矩阵 / Schur | $O(n^3)$ | 一次到位，标准库实现 |
| SDA 倍增 | $O(n^3\log(1/\epsilon))$ | 最快最稳，需专门结构推导 |

**解的存在唯一性**：$(A,B)$ 可稳（stabilizable）且 $(A,Q^{1/2})$ 可检测（detectable）时，DARE 存在**唯一**的半正定 stabilizing 解 $S^*$，此时 $A-BK$ 的全部极点落在单位圆内。

## 三、从 LQR 到有限时域 LQR（Riccati 递推）

若时域有限，边界条件由终端代价给出（$S_N=Q_N$），DARE 退化为**后向递推**：

$$
K_k = (R + B^\top S_{k+1} B)^{-1} B^\top S_{k+1} A,\qquad
S_k = Q + A^\top S_{k+1} A - A^\top S_{k+1} B K_k
$$

此时增益是时变的，$u_k^* = -K_k x_k$。这个递推就是后面"五、Riccati-like backward sweep"在线性、无约束、二次情形下的全部内容——OSQP 的 MPC 是"约束 + 线性"，同一套递推在 iLQR/SQP 里则用来解每个迭代的 LQ 子问题。

## 四、"离线求解"的准确含义

**离线阶段**：解一次 DARE 得到 $S^*$，再算一次 $K=(R+B^\top S^* B)^{-1}B^\top S^* A$。

**在线阶段**：每步只做

$$
u_k = -K x_k
$$

一次 $m\times n$ 的矩阵-向量乘。**在线没有任何迭代、没有 QP 求解器、没有矩阵求逆、也不需要存储整条轨迹**——对比 OSQP 的 MPC：每步要跑数十次 ADMM 迭代外加一次 LDLᵀ 分解。因此 LQR 可以跑在 MCU 上、跑到 1 kHz 以上，且每步耗时是**确定**的（无迭代、无分支），这是它在嵌入式控制中长期不可替代的原因。

**代价**：因为要离线算出常值 $K$，就必须假设 $(A,B)$ 不变、目标不变、约束可忽略。

## 五、$A,B$ 变化时怎么办

| 变化的东西 | 是否重算 $K$ | 处理方式 |
|---|---|---|
| 参考点 / 目标 $x_r$ 变化 | **不需要** | 改写为 $u=-K(x-x_r)+u_{ff}$，前馈 $u_{ff}$ 由稳态关系 $x_r=(A-BK)x_r+Bu_{ff}$ 解出 |
| 线性化工作点漂移 | 需要 | 增益调度 / LPV / SDRE |
| 系统本身时变、参数漂移 | 需要 | 在线重解或自适应 LQR |
| 状态带噪声、不可全测 | $K$ 不变 | LQG = LQR + Kalman 滤波器（分离原理，可独立设计） |

**参考跟踪的标准写法**（覆盖绝大多数"目标变了"的场景，不需要重算 Riccati）：

$$
u = -K(x-x_r) + N_u r
$$

其中 $N_u$ 由"稳态无误差"条件解出（MATLAB 的 reference-tracking form 用 `dlqr` 先算 $[K,S]$，再算 $N_x,N_u$）。

**$A,B$ 真变化时的四条出路**：

1. **增益调度 / 分段线性 / LPV**：在工作点网格 $\{\rho_i\}$ 上各自线性化得到 $\{A_i,B_i\}$，离线算出 $\{K_i\}$，在线按调度变量 $\rho$ 插值。典型：四旋翼在悬停附近按姿态与速度分箱、车辆按纵速分箱做横向 LQR。
2. **SDRE / iLQR / NMPC（在线重解）**：把非线性系统写成 state-dependent coefficient form $x_{k+1}=A(x)x_k+B(x)u_k$，每个控制周期用当前状态重新做一次 Riccati 后向递推，甚至迭代到收敛。**iLQR 本质上就是"每个时刻都重算 LQR"的在线版本**，代价是每步 $O(N)$ 的计算量，需要实时预算。
3. **自适应 LQR（self-tuning regulator）**：在线辨识 $(\hat A,\hat B)$（递推最小二乘 / 子空间辨识），再在线解 DARE 更新 $K$。注意"辨识 + 控制"的耦合会带来稳定性问题，必须保证足够的激励。
4. **升级为 MPC**：如果变的是**约束**（输入饱和、关节限位、安全边界）而非模型，$A,B$ 其实没变，此时应换成 MPC——**约束才是 LQR 与 MPC 的分水岭**。

## 六、LQR 的适用范围

**前提**：系统可近似线性化、无硬约束、状态可测（或可观测）、目标是在平衡点/标称轨迹附近调节与稳定。

- **姿态与平衡**：倒立摆、平衡车、四旋翼悬停定点、卫星姿态稳定、火箭与导弹的小偏差线性化通道。
- **伺服与电力电子**：直流电机转速/位置环、逆变器电流环、Buck/Boost 变换器电压环、机器人关节伺服（前馈 + PD + LQR 内环）。
- **轨迹跟踪（局部）**：飞机/车辆小舵角下的横向-纵向控制、机械臂末端在标称轨迹附近的线性化控制 + 前馈。
- **作为组件（最常见的用法）**：
  - **MPC 的终端代价与终端集**：用 DARE 解出的 $S^*$ 作为 $\ell_f(x)=\frac12x^\top S^*x$ 是保证 MPC 递归可行性与闭环稳定性的标准手段；
  - **iLQR / DDP / SQP 的内层**：Riccati 递推就是每个迭代 LQ 子问题的求解器（本文档后面 OCS2 那节）；
  - **强化学习的线性二次基线**、**轨迹优化器的 warm start**。

**优点**：无在线优化、可证明稳定性、计算量确定，而且有内生的鲁棒性余量（单输入情形下相位裕度 $\ge 60^\circ$、增益裕度无穷大、幅值裕度 $1/2\sim\infty$）。

**失效场景**：
- **有硬约束**：输入饱和、关节限位、安全边界无法表达——这是 MPC 存在的理由；
- **大范围非线性**：$K$ 只在平衡点附近最优，离远了线性模型失效会**直接失稳**——这是 iLQR / NMPC 存在的理由；
- **模型不准或需要全状态**：依赖精确模型与全部状态（否则要配观测器/Kalman）。

# iLQR的本质是什么？

**iLQR = 在非线性系统上反复做 LQR（iterative LQR）。** 它是 DDP 的 Gauss–Newton 版本，也是 SQP 在"只线性化动力学、不二阶化动力学"下的特例。

## 一、iLQR的典型形式

$$
\begin{aligned}
\min_{x_{0:N},\,u_{0:N-1}}\quad & \sum_{k=0}^{N-1}\ell(x_k,u_k) + \ell_f(x_N) \\
\text{s.t.}\quad & x_{k+1} = f(x_k,u_k),\\
& x_0 = \bar{x}.
\end{aligned}
$$

动力学 $f$、阶段代价 $\ell$、终端代价 $\ell_f$ 均非线性，但没有不等式约束。与上面的（无限时域 LQR）相比：动力学与代价是非线性的、时域是有限的；与 NMPC 相比：**缺少 $g(x_k,u_k)\le 0$ 这类硬约束**。

## 二、iLQR的求解：线性化 + 二次化 + Riccati

在名义轨迹 $\{\bar{x}_k,\bar{u}_k\}$ 上反复迭代：

1. **Forward rollout**：用真实非线性动力学从 $\bar{x}_0$ 积分得到 $\{\bar{x}_k\}$；
2. **线性化动力学**：$\delta x_{k+1} \approx A_k\delta x_k + B_k\delta u_k$，其中增量 $\delta x_k = x_k-\bar{x}_k$（iLQR 忽略 $f$ 的二阶项，故无 $d_k$ 偏置，这正是与 DDP 的唯一区别）；
3. **二次化代价**：得到 $Q_{xx},Q_{uu},Q_{ux}$ 与 $q_x,q_u$；
4. **Riccati backward pass**：得到 $\delta u_k = k_k + K_k\,\delta x_k$，$O(N)$ 复杂度；
5. **Forward pass + line search**：$u_k^{new} = \bar{u}_k + \alpha\,(k_k + K_k\delta x_k)$，用真实非线性动力学重新 rollout 并检查真实 cost 是否下降；
6. 重复直到收敛，取 $u_0$ 下发（receding horizon 时只取第一个）。

其细节即本文档后面"SQP中Riccati backward/forward pass 的具体步骤"。

### 2.1 代价的二次化（Quadratic approximation）

一维直觉：$\ell(x)\approx\ell(\bar x)+\ell'(\bar x)(x-\bar x)+\tfrac12\ell''(\bar x)(x-\bar x)^2$。多维情形就是对**状态和控制同时**做二阶泰勒展开。

**第一步：定义增量。**

$$
\delta x_k = x_k - \bar x_k,\qquad \delta u_k = u_k - \bar u_k
$$

其中 $(\bar x_k,\bar u_k)$ 是**当前迭代的名义轨迹**（由上一步 forward rollout 产生），它严格满足非线性动力学。

**第二步：动力学线性化。**

$$
A_k = \frac{\partial f}{\partial x}\Big|_{\bar x_k,\bar u_k},\qquad
B_k = \frac{\partial f}{\partial u}\Big|_{\bar x_k,\bar u_k},\qquad
\delta x_{k+1} = A_k\,\delta x_k + B_k\,\delta u_k
$$

iLQR 到这一步就停（**丢弃 $f_{xx},f_{xu},f_{uu}$**），因此是 Gauss–Newton 型；DDP 会把 $f$ 的二阶项带进 $Q_{uu}^Q,Q_{ux}^Q$ 等系数里，代价是多做 $n\times n\times m$ 的张量乘法。

**第三步：阶段代价的梯度与 Hessian。**

$$
q_k=\ell_x(\bar x_k,\bar u_k),\quad r_k=\ell_u(\bar x_k,\bar u_k),\quad
Q_k=\ell_{xx},\quad R_k=\ell_{uu},\quad P_k=\ell_{xu}
$$

（这里用 $\ell_{xx}$ 等符号，以区别于 Q-function 的 $Q_{xx}^Q$。）代入：

$$
\ell(\bar x_k+\delta x,\bar u_k+\delta u)
\approx
\underbrace{\ell(\bar x_k,\bar u_k)}_{\text{常数 }\ell_k}
+ q_k^\top \delta x + r_k^\top \delta u
+ \frac12
\begin{bmatrix}\delta x\\ \delta u\end{bmatrix}^\top
\underbrace{
\begin{bmatrix}\ell_{xx} & \ell_{xu}\\ \ell_{ux} & \ell_{uu}\end{bmatrix}
}_{\ell_{ux}=\ell_{xu}^\top}
\begin{bmatrix}\delta x\\ \delta u\end{bmatrix}
$$

**第四步：二次跟踪代价的具体数值。** 若

$$
\ell(x,u)=\tfrac12(x-x_r)^\top Q(x-x_r)+\tfrac12 u^\top R u
$$

则

$$
\ell_x=Q(\bar x_k-x_r),\quad \ell_u=R\bar u_k,\quad
\ell_{xx}=Q,\quad \ell_{uu}=R,\quad \ell_{xu}=0
$$

**若阶段代价本来就是二次的，二阶展开没有截断误差**，此时"二次化"只是把常数项与一次项补出来。只有当 $\ell$ 真非线性时才有近似误差——常见来源：四元数姿态误差、task-space 欧氏距离、$\cos\theta$ 型代价、log-barrier、神经网络代价。

**第五步：终端代价同样展开**（围绕 $\bar x_N$）：

$$
\ell_f(\bar x_N+\delta x_N)\approx \ell_f(\bar x_N)+\ell_{fx}^\top\delta x_N+\tfrac12\delta x_N^\top \ell_{fxx}\delta x_N
$$

$\ell_{fxx}$ 需 PD（至少 PSD），通常再加 $\lambda I$；否则终端 $S_N$ 非正定，backward pass 会把数值误差指数放大。

**第六步：整条轨迹的 LQ 子问题。**

$$
\begin{aligned}
\min_{\delta x,\delta u}\quad
& \frac12\delta x_N^\top \ell_{fxx} \delta x_N + \ell_{fx}^\top\delta x_N
+\sum_{k=0}^{N-1}\left(
\frac12
\begin{bmatrix}\delta x_k\\ \delta u_k\end{bmatrix}^\top
\begin{bmatrix}\ell_{xx} & \ell_{xu}\\ \ell_{ux} & \ell_{uu}\end{bmatrix}
\begin{bmatrix}\delta x_k\\ \delta u_k\end{bmatrix}
+\begin{bmatrix}\ell_x\\ \ell_u\end{bmatrix}^\top
\begin{bmatrix}\delta x_k\\ \delta u_k\end{bmatrix}
\right)\\
\text{s.t.}\quad
& \delta x_{k+1}=A_k\delta x_k+B_k\delta u_k,\qquad \delta x_0=0
\end{aligned}
$$

$\delta x_0=0$ 是因为起点用测量值固定（对初值不加优化）；DDP 的动力学多一项偏置 $+d_k$，iLQR 没有。

存储量：$\ell_{xx},\ell_{xu},\ell_{uu}$ 每个时刻一套且**时变**，总计 $O(N(n^2+nm+m^2))$。

### 2.2 Riccati backward pass 的完整推导

**思路一句话**：状态 $\delta x$ 被动力学"绑住"（等式约束），而控制 $\delta u$ **是自由的**。所以逆推时用动态规划，把"$\delta x$ 的最优剩余代价"吸收进值函数，只对 $\delta u$ 做无约束极小化——这一步永远有闭式解，$K_k$ 就是那次求导的结果。

**归纳假设（值函数是二次型）：**

$$
V_k(\delta x_k) = \frac12 \delta x_k^\top S_k\,\delta x_k + s_k^\top \delta x_k + c_k
$$

边界（终端）：$S_N=\ell_{fxx}$，$s_N=\ell_{fx}$，$c_N=\ell_f(\bar x_N)$。

**定义 Q-function（局部 Bellman 展开）：**

$$
Q_k(\delta x,\delta u)
= \ell(\bar x_k+\delta x,\bar u_k+\delta u)
+ V_{k+1}\!\left(A_k\delta x+B_k\delta u\right)
$$

**展开 $V_{k+1}$ 那一项**（所有系数的来源）：

$$
\begin{aligned}
V_{k+1}(A\delta x+B\delta u)
={}&\underbrace{\tfrac12\delta x^\top A^\top S_{k+1}A\,\delta x}_{xx}
+\underbrace{\delta x^\top A^\top S_{k+1}B\,\delta u}_{xu}
+\underbrace{\tfrac12\delta u^\top B^\top S_{k+1}B\,\delta u}_{uu}\\
&+\underbrace{s_{k+1}^\top A\,\delta x}_{x}
+\underbrace{s_{k+1}^\top B\,\delta u}_{u}
+c_{k+1}
\end{aligned}
$$

**加上阶段代价的 $\ell_{xx},\ell_{xu},\ell_{uu},\ell_x,\ell_u$，得到 Q-function 的五个系数：**

$$
\boxed{
\begin{aligned}
Q_{xx}^Q &= \ell_{xx} + A_k^\top S_{k+1} A_k \\
Q_{uu}^Q &= \ell_{uu} + B_k^\top S_{k+1} B_k \\
Q_{ux}^Q &= \ell_{ux} + B_k^\top S_{k+1} A_k \\
Q_x^Q &= \ell_x + A_k^\top s_{k+1} \\
Q_u^Q &= \ell_u + B_k^\top s_{k+1}
\end{aligned}}
$$

DDP 里 $Q_x^Q,Q_u^Q$ 要额外加 $A^\top S_{k+1}d_k$、$B^\top S_{k+1}d_k$，$Q_{uu}^Q$ 等还要加 $f$ 的二阶项；文档后面 OCS2 那节的公式正是带 $d_i$ 的版本。

**对 $\delta u$ 求导置零 —— $K_k$ 的诞生：**

$$
\frac{\partial Q_k}{\partial \delta u}=Q_{uu}^Q\,\delta u+Q_{ux}^Q\,\delta x+Q_u^Q=0
$$

这是关于 $\delta u$ 的**无约束二次型**，当 $Q_{uu}^Q\succ0$（通常由 $R\succ0$ 保证）时解唯一：

$$
\boxed{\;
\delta u_k^* = \underbrace{-\left(Q_{uu}^Q\right)^{-1} Q_u^Q}_{\textstyle k_k}
\;+\;
\underbrace{-\left(Q_{uu}^Q\right)^{-1} Q_{ux}^Q}_{\textstyle K_k}\,\delta x_k
\;}
$$

对照 OCS2 那节的 $K_k=-(R_k+B_k^\top S_{k+1}B_k)^{-1}(P_k^\top+B_k^\top S_{k+1}A_k)$：只要令 $\ell_{uu}=R_k$、$\ell_{ux}=P_k^\top$，两者**完全同一个公式**。

**$K_k$ 的三个来源**：

1. $Q_{uu}^Q=\ell_{uu}+B^\top S_{k+1}B$：控制代价 + 控制对未来代价的影响。求逆它相当于"归一化"——把代价单位换算到控制量单位。$B$ 越大（控制越有效），该矩阵越大，$K$ 越小。
2. $Q_{ux}^Q=\ell_{ux}+B^\top S_{k+1}A$：控制与状态的**耦合项**。$B^\top S_{k+1}A$ 的含义是"控制通过 $B$ 影响下一时刻状态，该状态再通过 $A$ 影响未来的代价权重 $S_{k+1}$"。
3. $Q_u^Q=\ell_u+B^\top s_{k+1}$：当前控制代价的梯度 + 未来的一次项（跟踪参考的那部分）。

因此 $K_k$ 就是 **LQR 增益 $K=(R+B^\top SB)^{-1}B^\top SA$ 的时变、带 cross-term 的版本**；若 $\ell_{xu}=0$、$\ell_{xx}=Q$（纯跟踪代价），立即退化成 LQR 的 $K$。

**代回得到值函数更新（闭合归纳）：**

$$
\boxed{\;
S_k=Q_{xx}^Q+Q_{xu}^Q K_k,\qquad
s_k=Q_x^Q+Q_{xu}^Q k_k\;}
$$

数值上更推荐对称写法（避免误差累积破坏对称性）：

$$
S_k = Q_{xx}^Q + K_k^\top Q_{uu}^Q K_k = Q_{xx}^Q - Q_{xu}^Q\left(Q_{uu}^Q\right)^{-1}Q_{ux}^Q
$$

**整条 backward pass**：

```pseudo
S, s = lf_xx, lf_x                    # 终端条件
for k = N-1 down to 0:
    # 组装 Q 系数
    Q_xx = l_xx[k] + A_k' * S * A_k
    Q_uu = l_uu[k] + B_k' * S * B_k
    Q_ux = l_ux[k] + B_k' * S * A_k
    Q_x  = l_x[k]  + A_k' * s
    Q_u  = l_u[k]  + B_k' * s

    # 正定化（等价于加 (lambda/2)||du||^2）
    Chol = cholesky(Q_uu + lambda * I)

    # 增益
    k_k = -Chol.solve(Q_u)
    K_k = -Chol.solve(Q_ux)

    # 值函数更新（对称写法，用未正则化的 Q_uu）
    S = Q_xx + K_k' * Q_uu * K_k
    s = Q_x  + K_k' * Q_uu * k_k

    store(k_k, K_k)
```

**为什么是 $O(N)$**：每个时刻只需对 $m\times m$ 的 $Q_{uu}^Q$ 做一次 Cholesky，再与 $n\times n$ 矩阵做几次乘加，全程不出现 $Nn\times Nn$ 的大规模求解。Riccati 本质上是**把带状 KKT 系统做块消元的递归实现**。

**为什么必须同时有 $K_k$ 与 $k_k$**：$k_k$ 是开环方向（把名义轨迹往哪挪），$K_k$ 是**闭环反馈增益**。forward pass 中的 $\delta x$ 来自真实非线性 rollout，而非模型预测，必须靠 $K_k$ 实时纠偏；这也使 iLQR 天然具备抗扰能力——等价于一个随时间变化的 LQR 控制器。

## 三、iLQR的收敛判据

实际实现里通常同时启用判据 A 与 B，再用 D 兜底。

**判据 A：代价相对下降量（最通用）**

$$
\frac{J_{\text{old}}-J_{\text{new}}}{\max(1,\,|J_{\text{old}}|)}<\epsilon_J
\qquad(\epsilon_J=10^{-6}\sim10^{-4})
$$

$J$ 必须是 forward pass 用真实非线性动力学 rollout 后算出的**真实代价**，而不是二次模型的预测值。

**判据 B：KKT / 梯度范数（最有信息量，且免费）**

backward pass 算出的开环增量本身就是梯度的预条件形式：

$$
Q_{u,k}^Q = \frac{\partial J}{\partial u_k}\Big|_{\text{沿轨迹}},\qquad
k_k=-\left(Q_{uu,k}^Q\right)^{-1}Q_{u,k}^Q
$$

因此

$$
\max_k\|k_k\|_\infty<\epsilon_k
\quad\text{或}\quad
\sqrt{\textstyle\sum_k\|Q_{u,k}^Q\|^2}<\epsilon_g
$$

即表示 $\nabla_u J\to0$，轨迹达到（局部）驻点。注意 $Q_u^Q$ 是**总导数**：它已把动力学当作约束、把状态的隐式依赖算进去，不是偏导数 $\ell_u$。

**判据 C：模型与实际改善的一致性（信任比）**

$$
\rho=\frac{J_{\text{old}}-J_{\text{new}}}{\Delta J_{\text{pred}}},
\qquad
\Delta J_{\text{pred}}=-\sum_k\left(Q_{u,k}^\top k_k+\tfrac12 k_k^\top Q_{uu,k} k_k\right)
$$

$\rho\approx1$ 说明二次模型与真实代价吻合，可放大步长；$\rho\ll1$ 或为负说明离线性化点太远，需加大正则化或拒绝该步。

**判据 D：工程性停止条件**

- `max_iter` 达到上限（常见 20~50 次）；
- **实时 MPC 的时间预算耗尽**——不等收敛，直接下发当前最好轨迹（warm start 后通常 1~5 次迭代已足够）；
- backward pass 中 $Q_{uu}^Q$ 加 $\lambda I$ 到上限仍非正定 → 当前线性化点病态，停止；
- line search 连续失败（$\alpha<\alpha_{\min}$）→ 无法继续改进，停止。

**收敛速度与正则化调度**：iLQR 在解附近是 superlinear（近似 quadratic）收敛，通常 10~30 次迭代到机器精度；远离解时靠 line search 保证下降、靠 $\lambda$ 自适应维持稳定。

```pseudo
if improved:                                    # 实际代价下降
    lambda = max(lambda / 10, lambda_min)       # 信任模型，收紧正则
else:
    lambda = min(lambda * 10, lambda_max)       # 模型失准，加大正则（趋近梯度下降）
```

**重要提醒**：iLQR 收敛到的是**局部**最优 / KKT 驻点。非凸问题（绝大多数机器人任务）依赖初值——这正是 MPC 每周期 warm start（用上一周期解）+ 短 horizon 得以工作的原因，也是轨迹优化需要多初值/随机重启的原因。

## 四、iLQR的思想与局限

- **思想**：把"非线性最优控制"拆成一串"LQ 子问题"，每个子问题都有闭式解，无需通用 QP/SQP 求解器；
- **优点**：复杂度 $O(N)$，收敛快，非常适合高维（如机械臂）问题；
- **局限**：只有局部收敛保证（靠 line search 与 $Q_{uu}$ 正定化 $\lambda I$ 维持下降）；**无法直接处理状态/输入硬约束**，需借助投影、惩罚或增广拉格朗日（如 ALTRO / TrajOpt 的做法）。

# NMPC的本质是什么？

**NMPC = 保留 MPC 的"在线滚动优化"框架，但动力学 $f$、代价 $\ell$、约束 $g$ 全部非线性化。**

## 一、NMPC的典型形式

$$
\begin{aligned}
\min_{x_{0:N},\,u_{0:N-1}}\quad & \sum_{k=0}^{N-1}\ell(x_k,u_k) + \ell_f(x_N) \\
\text{s.t.}\quad & x_{k+1} = f(x_k,u_k),\\
& g(x_k,u_k) \le 0,\\
& x_0 = \bar{x}.
\end{aligned}
$$

与本文档开头 MPC 形式的逐项对应关系：

- $x_{k+1}=Ax_k+Bu_k \;\to\; x_{k+1}=f(x_k,u_k)$（非线性动力学，**必须**前向积分而非矩阵乘）；
- $(x_k-x_r)^\top Q(x_k-x_r)+u_k^\top R u_k \;\to\; \ell(x_k,u_k)$（一般代价，通常只在当前轨迹处做二次近似）；
- $x_{\min}\le x_k\le x_{\max}$、$u_{\min}\le u_k\le u_{\max} \;\to\; g(x_k,u_k)\le 0$（一般非凸不等式约束，线性化后进入 QP）；
- 仍然是 **receding horizon**：每个控制周期只执行 $u_0^*$，用新测量的状态重新求解。

## 二、NMPC的求解：SQP / 内点 + Riccati

NMPC 既没有 LQR 的解析解，也不能（像线性 MPC 那样）一次性写成标准 QP，只能**逐次逼近**：

1. **Forward rollout**：由当前控制序列 $u_{0:N-1}$ 积分出名义轨迹 $\{x_k,u_k\}$；
2. **局部展开**：线性化动力学与约束、二次近似代价，得到一个 **QP 子问题**（决策变量为增量 $\delta x,\delta u$）；
3. **解这个 QP**（见下文 2.3 的"内层"）；
4. **Line search**：$u_k^{new}=u_k+\alpha\,\delta u_k$，保证真实 cost 或 merit function 下降；
5. **迭代**直到收敛，输出最优控制序列与状态轨迹，只下发 $u_0^*$（receding horizon）。

OCS2 走的就是 SLQ / SQP + Riccati 这条路。

这正是它与"OSQP 解线性 MPC"在工程上的根本差异：**一个是外层 SQP 迭代 + 内层 Riccati，一个是单次 QP + ADMM 迭代。**

### 2.1 "两层"的准确划分

"分内外层"不是指"内层管无约束、外层管约束"，正确的划分是：

> **外层 = 求解非线性问题的迭代（SQP / DDP / iLQR），每次迭代把非线性的最优控制问题近似成一个 QP 子问题；**
> **内层 = 单独这一个 QP 自己怎么解（Riccati / active-set / 内点 / ADMM）。**

两层是**正交**的：外层负责非线性（决定迭代几次），内层负责单个子问题（决定用什么算法）。**外层为什么必须迭代**：$f,\ell,g$ 全非线性，没有解析解，也不能一次性写成标准 QP；模型只在当前名义轨迹附近有效，所以只能"展开 → 解 QP → 检验 → 移动轨迹 → 重新展开"。

### 2.2 内层为什么优先用 Riccati，而不是直接把稀疏 QP 丢给 OSQP

| 对比项 | Riccati 递推（结构化内层） | 直接丢给 OSQP / 稀疏 QP 求解器 |
|---|---|---|
| 复杂度 | $O(Nn^3)$，与 horizon $N$ **线性** | 稠密 $O((Nn)^3)$；利用稀疏性通常也仍是超线性 |
| 内存 | **完全不构造** $Nn\times Nn$ 的 $P$ 与 $A_c$ | 需显式构造稀疏块矩阵（见文档前面那张 $A_c$） |
| 副产品 | 直接得到时变**反馈律** $\delta u_k=k_k+K_k\delta x_k$ | 只给数值解，没有策略 |
| 迭代性 | 是 LQ 子问题的**精确解**，一次 pass 到位 | ADMM / 内点在内层还要跑数十次迭代 |

三条具体理由：

1. **结构利用**：QP 的 KKT 矩阵是块三对角（只有相邻时刻耦合）。Riccati 递推就是这个带状系统的**精确块消元 / 块 LDL 分解**，把 $O(N^3)$ 降为 $O(N)$，因此 $N$ 可以取到几百上千。
2. **内存与实时性**：不必构造巨大的 Hessian 与约束矩阵，适合嵌入式实时。
3. **必须拿到 $\delta u=k+K\delta x$**：这个 $K$ 是闭环稳定性、抗扰以及 forward pass 纠偏的基础。**QP 求解器给你 $u^*$，只有 Riccati 给你策略 $\pi(x)$。**

### 2.3 内层如何处理不等式约束

Riccati 只能解**无不等式约束**的 LQ 子问题。一旦有 $g(x,u)\le0$，就必须在外面再套一层来处理约束——这才是"分层"的来源：

- **(a) Active-set / 投影**：猜一个活跃集，把活跃约束当**等式**处理，用 Riccati 解，再检查约束是否被违反/该被释放，迭代更新活跃集（ALTRO、DDP with box constraints）。
- **(b) 算子分裂 / ADMM**：无约束 Riccati 步与"投影到约束集"交替进行（ALTRO 的增广拉格朗日版本）。
- **(c) 内点 / barrier**：引入松弛变量 + log barrier，把不等式变成等式，于是又回到"Riccati + 外层 barrier 参数 Newton"。
- **(d) 放弃 Riccati**：直接把稀疏 QP 交给 OSQP / qpOASES——实现简单通用，但丢掉 $O(N)$ 与反馈增益。

### 2.4 两处"两层"不要混淆

| | 线性 MPC（前面 OSQP 那节） | NMPC（OCS2 这节） |
|---|---|---|
| 非线性处理 | **没有**（模型本来就线性） | 外层 SQP/DDP 迭代若干次 |
| 每个控制周期的 QP 数 | **一个** | **一串**（每次 SQP 迭代一个） |
| QP 怎么解 | ADMM：x-update（解线性方程组，LDLᵀ 分解）+ z-update（投影） | 内层 Riccati + 约束处理（投影 / active-set / 内点） |
| 得到的东西 | 数值最优解 $z^*$ | 数值最优解 + **时变反馈律 $K_k$** |

即：**OSQP 的"两层"是 ADMM 的 x-update / z-update，整体上只解一个 QP；NMPC 的"两层"是外层 SQP 迭代 / 内层单个 QP 的解法。** 两者处于不同层面。

### 2.5 Line Search 的含义与做法

**要解决的问题**：QP 给出的 $\delta u$ 是**线性化 + 二次近似模型**下的最优步。模型只在名义轨迹附近有效，走满步（$\alpha=1$，牛顿步）很可能让**真实代价反而上升**（非线性失配导致的 overshoot）。

**中心思想**：**方向由 QP 定，步长由真实代价检验定。** 找一个 $\alpha\in(0,1]$，使得用**真实非线性动力学**算出的代价确实下降：

$$
J(u+\alpha\,\delta u) < J(u)
$$

**基本流程（backtracking）**：

```pseudo
alpha = 1.0
J_old = J(u_nom)                                    # 真实非线性代价
while alpha > alpha_min:
    x_new, J_new = rollout(u_nom + alpha * delta_u) # 真实动力学前向积分
    if J_new < J_old - c1 * alpha * grad_dot_d:     # 满足 Armijo
        accept; u_nom = u_nom + alpha * delta_u; break
    alpha *= beta                                   # 收缩，beta 常取 0.5
else:
    reject; lambda *= 10                            # 加大正则化，重做 backward pass
```

**Armijo 条件**（保证"下降足够快"，而不只是"下降了一点点"）：

$$
J(u+\alpha d) \le J(u) + c_1\,\alpha\,\nabla J^\top d,\qquad c_1\approx10^{-4}
$$

**三种常见变体**：

1. **整条轨迹共用一个 $\alpha$**（最朴素的 backtracking）：只需 rollout 一次，精度较粗。
2. **Tassa 式 iLQR**：只缩放**开环项**，反馈项保持**满增益**：
   $$
   u_k^{new}=\bar u_k+\alpha\,k_k+K_k\,\delta x_k
   $$
   理由是 $K_k\delta x_k$ 是**尺度无关的稳定/纠偏项**，一起缩放会削弱闭环稳定性且可能违反约束。对应的预测改善量有解析式（无需额外 rollout）：
   $$
   \Delta J_{\text{pred}}=-\sum_k\left(\alpha\,Q_{u,k}^\top k_k+\tfrac12\alpha^2 k_k^\top Q_{uu,k}k_k\right)
   $$
   令 $\frac{d}{d\alpha}\Delta J_{\text{pred}}=0$ 可先解出"模型最优步长" $\alpha^*$ 作为初值。
3. **Trust region（DDP / BSA / TrajOpt）**：不缩步长，而是限制 $\|\delta u\|\le\Delta$，用信任比调整半径：
   $$
   \rho=\frac{J_{\text{old}}-J_{\text{new}}}{\Delta J_{\text{pred}}}
   $$
   $\rho\approx1$ → 放大 $\Delta$；$\rho<0$ → 缩小或拒绝该步。

**与正则化 $\lambda I$ 的关系**：两者是同一件事的两个旋钮。$\lambda I$ 调**方向**（$Q_{uu}\leftarrow Q_{uu}+\lambda I$ 让步长更保守、更接近梯度下降方向），$\alpha$ 调**步长**（沿给定方向走多远）。很多实现只保留其一：Tassa 自适应 $\lambda$ 而固定 $\alpha$，OCS2/ALTRO 用 line search 配 merit function。

**带约束时的特殊处理**：此时"真实代价"不是唯一标尺（可能出现"代价降了但约束违反更严重"），因此用 **merit function**（真实代价 + 约束罚项）或直接比较 **KKT 违反度**；ALTRO 就是对外层罚参数做 line search。

**作用与意义**：
- 把"局部模型最优"变成"**真实单调下降**"，是 iLQR/SQP 全局化（globalization）的关键；
- 提供天然的失败信号：$\alpha$ 退化到下限说明无法继续改进 → 要么加大 $\lambda$、要么停止（实时 MPC 直接下发当前最好解）；
- 优化理论上，它把可能发散的"牛顿法"变成保证下降的"阻尼牛顿法 / Levenberg–Marquardt"。

## 三、统一视角

$$
\underbrace{\text{LQR}}_{\text{线性/二次/无约束/}\infty}
\;\xrightarrow{\;\text{加约束、限时域}\;}\;
\underbrace{\text{线性MPC}}_{\text{QP + OSQP}}
\;\xrightarrow{\;\text{非线性化}\;}\;
\underbrace{\text{NMPC}}_{\text{SQP + Riccati}}
$$

- **LQR**：解析解，一个常值反馈增益 $u=-Kx$；
- **线性 MPC**：每个周期解一个 QP（ADMM，如 OSQP）；
- **iLQR**：非线性 + 无硬约束，反复解 LQ 子问题（Riccati 前后向）；
- **NMPC**：非线性 + 有约束，SQP/内点在线求解，外层迭代、内层 Riccati。

一句话总结：**LQR 是 iLQR 的退化情形（系统本就线性、只迭代一次），iLQR 是 NMPC 的内层无约束求解器，而 NMPC 是线性 MPC 把模型换成非线性后的推广。**

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

其中：
- $x^{k+1}$：第 $k+1$ 次 ADMM 迭代中待求解的**原始变量**（在 MPC 语境下即堆叠的状态与控制序列 $z$）；
- $H$：原始 QP 的 **Hessian 矩阵**（即上文的 $P$，二次项系数矩阵）；
- $g$：原始 QP 的**梯度/线性项**（即上文的 $q$）；
- $\rho$：ADMM 的**惩罚参数**（step-size），控制约束违反的惩罚力度；
- $A$：**约束矩阵**（即上文的 $A_c$，包含动力学等式与状态/输入上下界投影）；
- $z^k$：第 $k$ 次迭代的 ADMM **辅助变量**（松弛变量）；
- $u^k$：第 $k$ 次迭代的**缩放对偶变量**（scaled dual variable）。

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