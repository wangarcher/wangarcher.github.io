---
layout: post
title:  "Interview(Planning)"
subtitle: "规划相关"
date:   2025-11-23 21:48:00
categories: [jotting, review]
---

# 路径规划算法笔记(PRM / RRT / A* / MINCO)

## 一、PRM(概率路线图,Probabilistic Roadmap)

采样类方法,适合静态环境、可"先建图、后多次查询"(如机械臂的关节空间规划)。

**主要流程(建图 + 查询两阶段)**

1. 采样:在自由空间 `C_free` 中均匀随机撒 N 个点,做碰撞检测,剔除落在障碍物中的点;
2. 连边:对每个节点取 k 近邻(或半径 r 内的邻居),用局部规划器(通常就是直线插值)尝试连接,逐边做碰撞检测,通过则把这条边加入路线图;
3. 查询:把起点、终点分别连到路线图中距离最近的若干"可见"节点上;
4. 搜索:在路线图上跑 Dijkstra / A* 得到路径,再做平滑与后处理。

**要点**

- 概率完备:解存在时,采样数趋于无穷则以概率 1 找到解;但不最优;
- **一次性批量撒点**:PRM 是"批采样(batch sampling)"的代表——先把 N 个点全撒完,再统一连边建图;这与 RRT 每轮只采一个点的"增量式采样"形成对照(详见 2.2 节);
- 建图代价与查询代价分离,一次建图多次查询非常划算,单次查询场景不合适;
- 常见变体:
  - PRM*:连边半径随样本数收缩 + 重连,渐进最优;
  - Lazy PRM:查询阶段才做碰撞检测,失败时再检查相关边;
  - OBPRM:在障碍物边界附近采样,改善窄通道连通性。

## 二、RRT(快速扩展随机树)及主要变体

### 2.1 基础 RRT 的流程

1. 树 T 初始化为只有根节点 `x_start`;
2. 在自由空间中**随机采样一个点** `x_rand`;
3. 找树上离它最近的节点 `x_nearest = argmin ||x - x_rand||`;
4. 从 `x_nearest` 朝 `x_rand` 方向走固定步长 η 得到 `x_new`,即 steer 操作:

$$x_{new} = x_{nearest} + \eta \frac{x_{rand} - x_{nearest}}{\lVert x_{rand} - x_{nearest} \rVert}$$

5. 对边 `(x_nearest, x_new)` 做碰撞检测,无碰撞则把 `x_new` 加入树;
6. 若 `x_new` 到达终点区域(或与终点可直连),回溯父指针得到路径;
7. 否则循环 2-6,直到最大迭代次数。

**特点**:概率完备、参数少、适合高维空间与复杂约束;但路径随机性强、不平滑、不最优,通常作为初值或配合平滑使用。

### 2.2 `x_rand` 是怎么生成的?会不会一次撒一大片点?

先回答"会不会撒一大片":**基础 RRT 每轮只生成一个 `x_rand`,采完立刻用掉(去 steer、去扩展树上的一个节点),然后丢弃**。采样是逐次的、增量的(incremental sampling),第 k 次与第 k+1 次采样独立同分布,随机点的集合是慢慢"长"出来的,而不是先撒一大片再统一处理;看上去像撒了一大片,只是把所有轮次采过的点在图上画出来的视觉印象。

真正**一次性撒一大片点**的是批量采样(batch sampling)一族:

| 方法 | 采样方式 | 特点 |
| --- | --- | --- |
| PRM | 先撒 N 个点,再统一连边建图 | 一次建图,多次查询 |
| FMT* | 撒出点云后由起点做"波前"式扩展 | 高维表现好,单次查询 |
| BIT* | 批量撒点与启发式搜索交替进行 | 融合 Informed 采样与 A* 的聚焦能力 |
| RRT / RRT* | 每轮一个点,增量式扩展 | 单次查询,概率完备 / 渐进最优 |

`x_rand` 具体的生成过程(设配置空间 $\mathcal{C} \subset \mathbb{R}^n$):

1. **均匀采样(最基础的做法)**:每一维各取一个 `[0,1)` 上的随机数,再线性映射到该维的取值范围:

$$x_{rand} = x_{min} + r \odot (x_{max} - x_{min}), \quad r \sim \mathcal{U}([0,1)^n)$$

   如果配置空间本身就是单位立方体,那 $x_{rand}$ 就是每维一个 `rand()`。
2. **要不要检查 `x_rand` 本身?** 原理版本只对边 `(x_nearest, x_new)` 做碰撞检测;工程实现里通常先判断 `x_rand` 是否落在自由空间,不合法就丢弃重采——这等价于一次**拒绝采样(rejection sampling)**,可以省掉大量无意义的 steer 与碰撞检测。
3. **为什么采样是均匀的,树却总往空旷处长?** 因为每次只走一步 η:节点被选为最近邻的概率正比于它的 **Voronoi 区域体积**,空旷区域里的节点 Voronoi 体积大、被选中概率高,障碍物附近和边界上的节点概率低。这就是 RRT 的 Voronoi bias,也是它探索性的来源。
4. **常见偏置采样(对生成方式的改进)**:
   - Goal-bias:以概率 $p_b$(常见 0.05~0.1)直接令 `x_rand = x_goal`,其余情况均匀采样。最常用的加速启发式;
   - 障碍物偏置 / bridge sampling:专门在障碍物边界附近采样(例如取两个可行点的中点,或让两点朝对方走),用来打通窄通道;
   - Informed 采样:已有解之后把采样限制在椭球区域内(见 2.5 节);
   - 流形上的采样:旋转空间 $SO(3)$ 不能"每维一个随机数",要按 Haar 测度均匀采四元数(用 3 个独立均匀随机数构造);$\mathrm{SE}(3)$ 则用"平移均匀 + 四元数均匀"的组合;
   - 采的不一定是状态:kinodynamic RRT 直接采控制量 `u ~ Uniform(Box)`,再前向积分得到新节点,相当于把"随机方向 + 固定步长"换成"随机控制 + 固定时间"。

一句话总结:`x_rand` 的生成 = 在配置空间里做一次(可能带偏置的)随机采样,再用 steer 把它变成"朝这个方向的一小步";而"一次撒一大片点"属于 PRM / FMT* / BIT* 这类批采样方法。

### 2.3 RRT-Connect(双向贪心扩展)

1. 以起点、终点为根各建一棵树 `T_a`、`T_b`;
2. 轮流扩展:当前树朝 `x_rand` 走一步(extend);
3. 成功后,另一棵树朝刚生成的新节点执行 connect —— 贪心地连续行走,直到到达或撞到障碍;
4. 两棵树的节点相遇(距离小于阈值)即拼接路径成功,否则交换角色继续。

比单向 RRT 收敛快得多,是实践中最常用的 RRT 变体之一。

### 2.4 RRT*(渐进最优)——详解

**要解决的问题**:RRT 只保证概率完备(能找到解),不保证最优。RRT*(Karaman & Frazzoli, 2011)在扩展时多加两个操作——**选择父节点(ChooseParent)** 与 **重连(Rewire)**,使树在采样数 $n \to \infty$ 时以概率 1 收敛到最优解,即**渐进最优(asymptotically optimal)**。

**记号**:$c(x)$ 表示树上从根 `x_start` 到节点 $x$ 的累计代价;边的代价 $\mathrm{Cost}(x, y)$ 必须满足"可加性"(于是总代价是各边之和)。

**完整流程(伪代码)**:

```text
V ← {x_start};  E ← ∅;  c(x_start) ← 0
for n = 1 to N do
    x_rand ← SampleFree(n)                    # 采样(可带 goal-bias / Informed 偏置)
    x_nearest ← Nearest(V, x_rand)            # ① 最近邻(用 KD-tree 加速)
    x_new ← Steer(x_nearest, x_rand)          # 朝 x_rand 走一步 η
    if CollisionFree(x_nearest, x_new) then
        X_near ← Near(V, x_new, r(n))         # ② 半径 r(n) 内的邻域节点
        # ---- 选择父节点 ----
        x_min ← argmin_{x ∈ X_near, CollisionFree(x, x_new)} { c(x) + Cost(x, x_new) }
        V ← V ∪ {x_new}
        E ← E ∪ {(x_min, x_new)};  c(x_new) ← c(x_min) + Cost(x_min, x_new)
        # ---- 重连(Rewire) ----
        for x ∈ X_near do
            if c(x_new) + Cost(x_new, x) < c(x) and CollisionFree(x_new, x) then
                Parent(x) ← x_new;  c(x) ← c(x_new) + Cost(x_new, x)
                # 该节点的子孙到根的代价也需要一起更新(递归更新,或查询时沿父指针回溯)
```

**与 RRT 的三点区别**:

1. 扩展对象不同:RRT 里 `x_new` 的父节点只能是 `x_nearest`;RRT* 在邻域 `X_near` 里挑"累计代价 + 边代价"最小的;
2. 多了重连:如果"绕道 `x_new`"能让邻居节点更便宜,就把它的父指针改成 `x_new`(树的局部"改线");
3. 邻域半径必须随迭代收缩,这是收敛性的关键:

$$r(n) = \min\left\lbrace \gamma \left(\frac{\log n}{n}\right)^{1/d}, \eta \right\rbrace, \quad \gamma = 2\left(1 + \frac{1}{d}\right)^{1/d} \left(\frac{\mu(\mathcal{C}_{free})}{\zeta_d}\right)^{1/d}$$

   其中 $d$ 是状态维数,$\mu(\mathcal{C}_{free})$ 是自由空间体积,$\zeta_d$ 是 $d$ 维单位球体积。半径不能固定(固定半径会让图失去"渐近连通"性质),也不能收缩太快,否则该连的边连不上,最优性保证就丢了。

**为什么能渐进最优(证明思路)**:

1. **覆盖**:采样点在 $\mathcal{C}_{free}$ 中稠密;按上式收缩的邻域半径既保证每个点周围"总有邻居",又保证邻居数不发散,于是随机几何图渐近连通;
2. **逼近**:对任意 $\varepsilon > 0$,总存在一条"折线化"的最优解近似路径,代价不超过 $(1+\varepsilon)c^{*}$($c^{*}$ 为最优代价),这条路径上的节点在 $n$ 足够大时都会进入树中;
3. **单调改善**:重连只会让节点的累计代价变小(单调不增),配合半径收缩,树上的最优解收敛:

$$\lim_{n \to \infty} c_n = c^{*} \quad \text{a.s.}$$

**复杂度与代价**:

- RRT:每轮 $O(\log n)$ 的最近邻查询,总共 $O(n \log n)$;
- RRT*:每轮多了 $O(\log n)$ 个邻居的父节点选择与重连(每个候选边还要重新做一次碰撞检测),总复杂度仍是 $O(n\log n)$,但常数大得多,实际收敛慢;
- 实践建议:先跑一段 RRT-Connect 拿到初始解,再切到 RRT* 或 Informed RRT*;高维场景(如 7 维以上机械臂)邻居查询极贵,通常用 BIT* 或者直接放弃渐进最优。

**相关变体**:RRT#、RRT*-Smart(复用已发现路径附近的采样)、RRG(只重连、不保证树结构)、k-RRT*(用 k 近邻代替半径)。

### 2.5 Informed RRT*(把采样限制在椭球里)

1. 先用 RRT* 跑出一个可行解,记其代价 $c_{best}$;
2. 由于欧氏距离满足三角不等式,任何可能改进解的路径必然满足 $\lVert x - x_{start} \rVert + \lVert x - x_{goal} \rVert \le c_{best}$,于是把之后的采样限制在这个**超椭球**内(二维是椭圆,高维叫 prolate hyperspheroid);
3. 具体采样方式:先在 $d$ 维单位球内均匀采样 $x_{ball}$,再做线性变换:

$$x_{ellipse} = C \cdot \mathrm{diag}\left(\frac{c_{best}}{2}, \frac{\sqrt{c_{best}^2 - c_{min}^2}}{2}, \dots, \frac{\sqrt{c_{best}^2 - c_{min}^2}}{2}\right) x_{ball} + x_{center}$$

   其中 $c_{min} = \lVert x_{goal} - x_{start} \rVert$,$C$ 是把第一主轴旋转到从起点指向终点方向的旋转矩阵(可用 Householder 变换构造);
4. 每次找到更好的解,$c_{best}$ 变小,椭球自动收缩,采样越来越聚焦。

它不破坏渐进最优性(采样密度仍与"可能改进的区域"测度成正比),如今几乎是 RRT* 实现的标准配置;此外还有"用已有解做启发式从终点反向长树"等加速手段。

### 2.6 Kinodynamic RRT*(考虑动力学约束)

1. 状态扩展为 `x = (p, v, ...)`,边不再是直线,而是由动力学 $\dot x = f(x, u)$ 生成的轨迹段;扩展方式可以是前向积分(随机采控制),也可以解**两点边值问题(BVP)**;
2. 对**线性系统 + 二次代价**(即 LQR 类型)$\dot x = Ax + Bu$、$\int u^T R u dt$,BVP 有闭式解(借助可控性 Gramian 矩阵),因此两点之间"代价最小的边"可精确算出;
3. RRT* 的 ChooseParent 与 Rewire 要比较的是**各条 BVP 边的最优代价**(而不是欧氏距离);Informed 采样同样可以搬到状态空间;
4. 非线性系统没有闭式解,通常用数值打靶或直接前向积分——此时一般只剩下概率完备性,渐进最优会丢失。

是采样规划与最优控制的结合,适用于车辆、无人机这类不能"瞬移"的系统。

### 2.7 其他值得了解的变体

- Lazy RRT:延迟碰撞检测,先找几何可行候选解,再检查边上的碰撞;
- BIT*(Batch Informed Trees):批量采样 + 类 A* 的启发式图搜索,兼顾均匀覆盖与聚焦,是目前中高维下的主流之一;
- Goal-biased RRT:`x_rand` 以一定概率直接取目标点,最常用的加速启发式;
- RRT-Connect + shortcut 平滑:工程上最常用的组合(最短路径 + 折线简化);
- RT-RRT* / DRRT:面向动态环境的重规划。

## 三、A*(图搜索)——详解

**问题设定**:给定图 $G = (V, E)$,每条边的代价 $c(u, v) \ge 0$,给定起点 $s$ 与终点 $g$,求代价最小的路径。

**核心评价函数**:

$$f(n) = g(n) + h(n)$$

- $g(n)$:从起点到节点 $n$ 的**当前已知最小代价**(不断被更新);
- $h(n)$:启发式函数,估计 $n$ 到终点的**剩余代价**;
- $f(n)$:经过 $n$ 的这条路径的代价估计。A* 每轮永远展开 $f$ 最小的节点——$g$ 保证"不走冤枉路",$h$ 保证"朝目标方向去"。

**数据结构**:

- Open list:按 $f$ 排序的优先队列(二叉堆,或支持 decrease-key 的 Fibonacci 堆);
- Closed list:已展开节点的集合;
- `g[]` 表:当前最优代价;`parent[]` 表:用于回溯路径。

**完整流程**:

```text
g[s] ← 0;  f[s] ← h(s);  push(open, s)
while open ≠ ∅:
    n ← pop_min(open)                   # 弹出 f 最小的节点
    if visited[n]: continue             # 惰性删除:同一节点可能被压入多次
    visited[n] ← true
    if n == goal: return ReconstructPath(parent, n)
    for m in Neighbors(n):              # 栅格:4 / 8 / 26 邻域
        if not CollisionFree(n, m):     # 栅格地图中把碰撞检测放在生成邻居这里
            continue
        tentative ← g[n] + c(n, m)
        if m ∉ g or tentative < g[m]:
            g[m] ← tentative;  parent[m] ← n;  f[m] ← tentative + h(m)
            push(open, m)               # 不再做 decrease-key,直接重复压入
return FAILURE                          # open 为空 ⇒ 不可达
```

**几个工程上的关键细节**:

- **惰性删除**:压入时不做 decrease-key,而是把新记录再压一份;弹出时判断该记录是否已过期(用 `visited` 或比较 `f`),实现简单且常数小;
- **Tie-breaking**:$f$ 相同时**优先展开 $g$ 更大**的节点(即离终点更近的),能大幅减少扩展数量;实现上用小顶堆比较 `(f, -g)` 即可。在开阔的栅格地图上效果非常明显;
- **要不要重新打开 closed 节点**:若 $h$ 是一致(consistent)的,$m$ 在 closed 里就可以安全跳过;若 $h$ 只是可采纳而不一致,严格的图搜索最优需要允许把 $m$ 从 closed 移回 open(否则可能返回次优解);
- 栅格地图的常见配套:障碍物按机器人半径做**膨胀**、对角移动时检查"两个相邻格都不撞"、搜索完做**折线简化(shortcut)**,再交给后端轨迹优化。

**启发式函数(以二维栅格为例)**:

- 曼哈顿距离 $h = \lvert \Delta x \rvert + \lvert \Delta y \rvert$:适用于 4-邻域(步长 1);
- 对角距离(octile)$h = (\sqrt{2} - 2) \min(\lvert \Delta x \rvert, \lvert \Delta y \rvert) + \lvert \Delta x \rvert + \lvert \Delta y \rvert$:适用于 8-邻域(直行 1、对角 $\sqrt{2}$);
- 欧氏距离 $h = \sqrt{\Delta x^2 + \Delta y^2}$:适用于任意方向移动;
- 结论:$h$ 必须**不高估**真实剩余代价;估得越准,扩展的节点越少。极端情况——$h$ 恰好等于真实剩余代价时,只扩展最优路径上的节点。

**最优性(为什么可采纳就够了)**:

- 若 $h$ 可采纳,记最优代价为 $c^*$。由于 $f$ 永远不会高估经过该节点的任何路径(见 $f = g + h$ 的构造),当终点被弹出时,$g(goal)$ 必然已经等于 $c^*$;
- $h \equiv 0$ ⇒ 退化成 Dijkstra(向四周均匀扩散);
- 若把 $h$ 乘上 $\varepsilon > 1$,得到 **Weighted A***:解更快但只保证 $c \le \varepsilon c^*$;
- 若用 $\hat h = h / w$($w$ 为估大系数)则会变成不可采纳但更快——这也是实践中常用的"用一点最优性换速度"。

**复杂度**:

- 时间:扩展的节点数 $\times$ 每个节点的邻居数 $\times$ 堆操作 $O(\log N)$;扩展节点数强烈依赖 $h$ 的质量;
- 空间:$O(N)$,要存 open / closed / g / parent;
- 与贪心最好优先搜索(只按 $h$ 排序)对比:后者快得多,但完全没有最优性保证。

**常用变体**:

- 双向 A*:从起点、终点同时搜索,两端相遇后比较候选最优解(注意正确的终止条件);
- ARA* / Anytime A*:先用大 $\varepsilon$ 快速得到一个有界次优解,再逐步减小 $\varepsilon$ 并复用之前的搜索结果,做到"随时可停、解的质量随时间提高";
- D* / D* Lite:从终点反向搜索,环境变化时只增量更新受影响节点,适合动态重规划(机器人导航常用);
- JPS(跳点搜索):均匀栅格上利用路径对称性跳过大量同构节点,只扩展"跳点",不改变最优性,常见实现里能提速一个数量级;
- Hybrid A*:节点是连续状态 `(x, y, θ)`,用运动基元(Reeds-Shepp / Dubins 曲线)扩展,满足车辆非完整约束,是泊车与 DARPA Urban Challenge 的经典方案;
- Lattice Planner:离线生成状态格运动基元,在线做格点图搜索,常用于结构化道路。

## 四、MINCO(重点详解)

**出处**:*Geometrically Constrained Trajectory Optimization for Multicopters*(Zhepei Wang, Xin Zhou, Chao Xu, Fei Gao, IEEE T-RO 2022,arXiv:2103.00190,代码 GCOPTER)。它是 GCOPTER、RAPTOR、多机协同规划等系统的轨迹优化核心。

**要解决的问题**:前端(采样/搜索)给出一串路标点之后,**如何用最少的决策变量,表示一条既光滑、又能灵活变形以满足各种约束的多项式轨迹?** 传统做法有两个痛点:

1. 直接把每段多项式的系数 $\mathbf{c}$ 当决策变量:维数是 $2sM$ 量级,还要显式写出所有连续性等式约束,又大又慢;
2. 固定时间分配 $\mathbf{T}$:轨迹无法在时间上变形,容易次优,也难满足动力学约束。

MINCO 的答案:**只用中间路标点 $\mathbf{q}$ 与段时间 $\mathbf{T}$ 作为决策变量**,多项式系数由"控制量最小的最优轨迹"唯一确定,并且以**线性复杂度**恢复出来。

### 4.1 多段控制量最小化问题

考虑 $s$ 阶积分链($m$ 维平输出轨迹 $z(t)$,控制量 $v = z^{(s)}$),时间被 $t_0 < t_1 < \dots < t_M$ 分成 $M$ 段:

$$\min_{z(t)} \int_{t_0}^{t_M} v(t)^T \mathbf{W} v(t) dt, \quad \mathbf{W} \succ 0$$

约束为:控制量定义 $z^{(s)}(t) = v(t)$、首末边界条件 $z^{[s-1]}(t_0) = \bar{z}_o$、$z^{[s-1]}(t_M) = \bar{z}_f$,以及中间时刻的条件

$$z^{[d_i-1]}(t_i) = \bar{z}_i$$

其中 $z^{[s-1]}$ 表示 $(z, \dot z, \dots, z^{(s-1)})$ 的堆叠,$\bar{z}_i$ 是 $t_i$ 处**指定的前 $d_i$ 阶导数**。对无人机轨迹规划,最常用的是 $d_i = 1$:每个中间时刻只固定位置,即"路标点"。

- $s = 3$(最小 jerk):每段是 5 次多项式,对应 3 阶积分链(位置/速度/加速度为状态);
- $s = 4$(最小 snap):每段是 7 次多项式。

### 4.2 最优性条件(MINCO 的理论基础)

论文给出的充要条件(Theorem 2):一条轨迹是上述问题的**最优解**,当且仅当

1. 每一段都是 $2s-1$ 次多项式;
2. 满足首末边界条件与中间点条件;
3. 在中间时刻 $t_i$ 处,轨迹直到 $\bar{d}_i - 1$ 阶都连续可微,其中 $\bar{d}_i = 2s - d_i$;
4. 且此条件下的解**存在且唯一**。

这个结论很漂亮:把 $d_i = 1$(纯路标点)代进去,得到"轨迹在路标处 $2s-2$ 阶连续"——也就是**除了最高那一阶导数以外,所有阶导数都是连续的**。

- 传统做法需要"人为假设连续到第几阶"再建方程组,而这里连续阶数是**最优性的推论**;
- 以最小 jerk($s = 3$)为例:常规做法只保证 $C^2$,而最优解在路标点处 jerk、snap 也连续,曲线更光滑;
- 唯一性保证了后面的带状矩阵一定非奇异,也保证了参数化到系数的映射是光滑的。

### 4.3 系数求解:带状线性系统,$O(M)$

把每段写成多项式

$$p_i(t) = \mathbf{c}_i^T \beta(t - t_{i-1}), \quad t \in [t_{i-1}, t_i), \quad \beta(x) = (1, x, x^2, \dots, x^{2s-1})^T$$

每个中间时刻把"中间点条件 + 连续性条件"整理成一对块 $\mathbf{E}_i, \mathbf{F}_i \in \mathbb{R}^{2s \times 2s}$:

$$\mathbf{E}_i \mathbf{c}_i + \mathbf{F}_i \mathbf{c}_{i+1} = \left( \mathbf{D}_i^T, \mathbf{0}^T \right)^T$$

其中的等式约束既包含"轨迹在 $t_i$ 处取指定值 $\mathbf{D}_i$"(前 $d_i$ 行),又包含"直到 $\bar{d}_i - 1$ 阶连续"(后 $\bar{d}_i$ 行)。

把所有块堆起来,得到整个问题的最小二乘恒等形式(实为确定性方程组):

$$\mathbf{M}(\mathbf{T}) \mathbf{c} = \mathbf{b}(\mathbf{q})$$

```text
   ⎡ F0   O    O    ...   O     O   ⎤ ⎡ c1 ⎤     ⎡ D0   ⎤
   ⎢ E1   F1   O    ...   O     O   ⎥ ⎢ c2 ⎥     ⎢ D1   ⎥
   ⎢ O    E2   F2   ...   O     O   ⎥ ⎢ c3 ⎥  =  ⎢ 0    ⎥
   ⎢ ...                            ⎥ ⎢ .. ⎥     ⎢ ...  ⎥
   ⎣ O    O    O    ...   O    E_M  ⎦ ⎣ cM ⎦     ⎣ DM   ⎦
        M 是分块带状(三对角)矩阵       c              b
```

关键性质:

- $\mathbf{M}$ 是**分块带状(block-banded)**的:第 $i$ 个块行只涉及第 $i$、$i+1$ 段的系数,首尾分别由边界块 $\mathbf{F}_0$、$\mathbf{E}_M$ 构成;
- 由解的唯一性可知,$\mathbf{M}$ 对任意 $\mathbf{T} \succ \mathbf{0}$ 都非奇异;
- 用**带状 PLU 分解**求解,时间与空间复杂度都是 $O(M)$,而且**根本不需要显式写出代价函数**;
- 于是得到映射

$$\mathbf{c} = \mathcal{M}(\mathbf{q}, \mathbf{T}): \mathbb{R}^{m \times (M-1)} \times \mathbb{R}_{>0}^{M} \to \mathbb{R}^{2Ms \times m}$$

这就是 MINCO 参数化:决策变量从 $2Ms$ 个系数,压缩成"$m(M-1)$ 个路标点坐标 + $M$ 段时间"。

### 4.4 MINCO 轨迹类与"可微层"视角

$$\mathcal{T}_{MINCO} = \lbrace p(t): [0, T] \to \mathbb{R}^m \mid \mathbf{c} = \mathcal{M}(\mathbf{q}, \mathbf{T}),\ \forall \mathbf{q}, \mathbf{T} \rbrace$$

任意定义在轨迹上的目标或约束 $\mathcal{K}(\mathbf{c}, \mathbf{T})$(代价、惩罚项、任务要求……)在 MINCO 上变成

$$\mathcal{W}(\mathbf{q}, \mathbf{T}) = \mathcal{K}(\mathcal{M}(\mathbf{q}, \mathbf{T}), \mathbf{T})$$

使用者只要把 $\partial\mathcal{K}/\partial\mathbf{c}$ 与 $\partial\mathcal{K}/\partial\mathbf{T}$ 传进来,就能拿到 $\partial\mathcal{W}/\partial\mathbf{q}$ 与 $\partial\mathcal{W}/\partial\mathbf{T}$;也就是说,MINCO 相当于一个**线性复杂度的可微层**。

### 4.5 解析梯度(伴随法,仍是 $O(M)$)

**对 $\mathbf{q}$ 求导**:对 $\mathbf{M}\mathbf{c} = \mathbf{b}$ 两边微分,得 $\partial\mathbf{c}/\partial q_{i,j} = \mathbf{M}^{-1} \partial\mathbf{b}/\partial q_{i,j}$,于是

$$\frac{\partial \mathcal{W}}{\partial q_{i,j}} = \mathrm{Tr}\left\lbrace \left(\frac{\partial \mathbf{b}}{\partial q_{i,j}}\right)^T \left(\mathbf{M}^{-T} \frac{\partial \mathcal{K}}{\partial \mathbf{c}}\right) \right\rbrace$$

而 $\partial\mathbf{b}/\partial q_{i,j}$ 只有一个非零元(第 $i$ 个路标点所在的行),所以只需**解一次伴随方程**

$$\mathbf{M}^T \mathbf{G} = \frac{\partial \mathcal{K}}{\partial \mathbf{c}}$$

就能拿到全部路标点的梯度:

$$\frac{\partial \mathcal{W}}{\partial \mathbf{q}} = \left( \mathbf{G}_1^T e_1, \dots, \mathbf{G}_{M-1}^T e_1 \right)$$

即取 $\mathbf{G}$ 中对应路标点的行。$\mathbf{M}^T$ 的带状分解可以直接复用 $\mathbf{M}$ 的分解结果($\mathbf{M}^T = \bar{\mathbf{L}}\bar{\mathbf{U}}\mathbf{P}^T$),全程 $O(M)$,**不需要任何显式求逆**。

**对 $\mathbf{T}$ 求导**:对 $T_i$ 微分得 $\frac{\partial \mathbf{M}}{\partial T_i}\mathbf{c} + \mathbf{M}\frac{\partial \mathbf{c}}{\partial T_i} = \mathbf{0}$,代入上面的 $\mathbf{G}$:

$$\frac{\partial \mathcal{W}}{\partial T_i} = \frac{\partial \mathcal{K}}{\partial T_i} - \mathrm{Tr}\left\lbrace \mathbf{G}_i^T \frac{\partial \mathbf{E}_i}{\partial T_i} \mathbf{c}_i \right\rbrace$$

其中 $\partial\mathbf{E}_i/\partial T_i$ 可以从 $\beta$ 的导数解析写出(只依赖 $T_i$),因此时间梯度同样是 $O(M)$。

结论:**从任意目标/约束到 $(\mathbf{q}, \mathbf{T})$ 的梯度传播是线性的**,每轮优化迭代都只花线性时间。

### 4.6 约束处理之一:时间约束用"微分同胚"消除

时间变量天然带约束($\mathbf{T} \succ \mathbf{0}$;总时长固定时还要 $\lVert \mathbf{T} \rVert_1 = T_\Sigma$)。用一个 $C^\infty$ 微分同胚把它映到无约束空间:

$$T_i = \frac{e^{\tau_i}}{1 + \sum_{j=1}^{M-1} e^{\tau_j}} T_\Sigma, \qquad T_M = T_\Sigma - \sum_{j=1}^{M-1} T_j, \qquad \boldsymbol{\tau} \in \mathbb{R}^{M-1}$$

逆映射很简单:$\tau_i = \ln(T_i / T_M)$。链式法则得到(记 $\partial J_q/\partial\mathbf{T} = (g_a^T, g_b)^T$):

$$\frac{\partial J}{\partial \boldsymbol{\tau}} = \frac{(g_a - g_b \mathbf{1}) \circ e^{[\boldsymbol{\tau}]}}{1 + \lVert e^{[\boldsymbol{\tau}]} \rVert_1} - \frac{\left( g_a^T e^{[\boldsymbol{\tau}]} - g_b \lVert e^{[\boldsymbol{\tau}]} \rVert_1 \right) e^{[\boldsymbol{\tau}]}}{\left(1 + \lVert e^{[\boldsymbol{\tau}]} \rVert_1 \right)^2}$$

其中 $e^{[\cdot]}$ 是逐元素指数,$\mathbf{1}$ 是全 1 向量。如果只要求 $\mathbf{T} \succ \mathbf{0}$(软时间正则),直接用 $\mathbf{T} = e^{[\boldsymbol{\tau}]}$ 就够了。

论文还证明了这类变换**不会引入新的局部极小,也不会消掉原有的局部极小**(一阶/二阶最优性条件被保持),所以可以放心地把约束"消除"掉、直接做无约束优化。

### 4.7 约束处理之二:位置约束用"光滑满射"消除

**(a) 球约束** $q_i \in \mathcal{B}(o_i, r_i)$(反立体投影 + 正交投影的复合):

$$f_\mathcal{B}(\xi) = o + \frac{2 r \xi}{\xi^T \xi + 1} \in \mathcal{B}(o, r), \qquad \xi \in \mathbb{R}^n$$

梯度:

$$\frac{\partial J}{\partial \xi_i} = \frac{2 r_i g_i}{\xi_i^T \xi_i + 1} - \frac{4 r_i (\xi_i^T g_i) \xi_i}{(\xi_i^T \xi_i + 1)^2}$$

若已有初值 $q_i$,可用局部逆映射回推 $\xi_i$:

$$\xi_i = \frac{r_i - \sqrt{r_i^2 - \lVert q_i - o_i \rVert_2^2}}{\lVert q_i - o_i \rVert_2^2} (q_i - o_i)$$

**(b) 多面体约束** $\mathcal{P}^\mathcal{H} = \lbrace x \mid \mathbf{A} x \preceq b \rbrace$(安全走廊的凸胞):先转成重心坐标 $q = v_0 + \hat{\mathbf{V}} w$,其中 $w \succeq 0, \lVert w \rVert_1 \le 1$;再用平方映射 $w = [x]^2$ 与上面的球映射复合:

$$f_\mathcal{H}(x) = v_0 + \frac{4 \hat{\mathbf{V}} [x]^2}{(x^T x + 1)^2} \in \mathcal{P}^\mathcal{H}, \qquad x \in \mathbb{R}^{\hat{n}}$$

梯度:

$$\frac{\partial J}{\partial \xi_i} = \frac{8 \xi_i \circ \hat{\mathbf{V}}^T g_i}{(\xi_i^T \xi_i + 1)^2} - \frac{16 g_i^T \hat{\mathbf{V}} [\xi_i]^2}{(\xi_i^T \xi_i + 1)^3} \xi_i$$

**意义**:路标点**按构造永远落在凸胞里**(位置约束被硬性满足),于是优化彻底变成无约束问题;而且这些映射同样不会引入多余的局部极小。

### 4.8 约束处理之三:连续时间约束用"时间积分惩罚"转录

速度上限、加速度上限、推力上下限、倾角上限这类约束,要求**对任意时刻 $t$ 都成立**,是无限维约束,不可能直接写进有限维 NLP。做法是定义**时间积分惩罚泛函**:

$$I_{\mathcal{G}}^k[p] = \int_0^T \max\left[\mathcal{G}(p(t), \dot{p}(t), \dots, p^{(s)}(t)), \mathbf{0}\right]^k dt, \qquad I_{\mathcal{G}}[p] = \chi^T I_{\mathcal{G}}^k[p]$$

- $k = 1$ 时是精确罚(但不光滑);$k = 3$ 时是光滑、严格凸的惩罚,工程上常用;
- 数值积分按段做(梯形法),第 $i$ 段取 $\kappa_i$ 个采样点:

$$I(\mathbf{c}, \mathbf{T}) = \sum_{i=1}^{M} T_i \kappa_i \sum_{j=0}^{\kappa_i} \bar{\omega}_j \chi^T \max\left[\mathcal{G}_\tau(\mathbf{c}_i, T_i, j/\kappa_i), \mathbf{0}\right]^k$$

其中采样函数 $\mathcal{G}_\tau(\mathbf{c}_i, T_i, \tau) = \mathcal{G}(\mathbf{c}_i^T \beta(T_i \tau), \dots)$。

**三个关键好处**:

1. **约束评估的分辨率 $\kappa_i$ 与决策变量维数解耦**:想提高约束精度只需加密采样,不需要增加优化变量;
2. 惩罚法**不需要可行的初值**(相比约束优化的一大优势);
3. 用平输出(微分平坦)把状态/输入约束写成轨迹导数的代数式,惩罚函数能直接吃进去。

### 4.9 总体优化问题与完整流程

$$\min_{\boldsymbol{\xi}, \boldsymbol{\tau}} J\left(\mathbf{q}(\boldsymbol{\xi}), \mathbf{T}(\boldsymbol{\tau})\right) + I\left(\mathbf{c}(\mathbf{q}(\boldsymbol{\xi}), \mathbf{T}(\boldsymbol{\tau})), \mathbf{T}(\boldsymbol{\tau})\right)$$

这是一个**无约束、低维、梯度解析可得**的 NLP,用 L-BFGS 求解。完整 pipeline:

1. **前端**:Informed RRT* / A* / Hybrid A* 在占据栅格或点云上搜一条引导路径(只考虑几何);
2. **走廊生成**:沿引导路径生成一串凸多面体(或球)覆盖自由空间,相邻两个凸胞的公共区域里安放路标点;
3. **初始化**:$\mathbf{q}$ 取引导路径上的路标点(也可先在走廊交集中做一次"距离最小化"得到更顺的初值);$\mathbf{T}$ 用梯形速度曲线或 TOPP 给初值;再用微分同胚反解出 $\boldsymbol{\tau}$;
4. **无约束优化**:每轮迭代 = 解一次带状系统得到 $\mathbf{c}$ → 计算 $J$ 与 $I$ 及梯度(伴随法 + 链式法则)→ L-BFGS 更新 $(\boldsymbol{\xi}, \boldsymbol{\tau})$;
5. **校验与后处理**:重采样检查动力学与安全性;若某些段超限,做**时间重分配**(放大对应 $T_i$)或与走廊生成交替迭代;最终输出位置/速度/加速度/姿态/推力的时间序列,交给控制器。

### 4.10 为什么值得重点学(面试速答)

**一分钟版本**:MINCO 是"控制量最小的多段多项式轨迹"这一族轨迹的参数化——决策变量只有**中间路标点 $\mathbf{q}$ 与段时间 $\mathbf{T}$**;给定 $(\mathbf{q}, \mathbf{T})$,最优轨迹唯一,其系数由一个分块带状线性系统在 $O(M)$ 内解出;目标对 $(\mathbf{q}, \mathbf{T})$ 的梯度用**伴随法**在 $O(M)$ 内回传;连续时间约束用**时间积分惩罚**从无限维转录成有限维,位置约束用**光滑满射**完全消除,时间约束用**微分同胚**消除;最后用 L-BFGS 解一个无约束 NLP。

**要点清单**:

- **时空解耦**:$\mathbf{q}$ 决定几何形状,$\mathbf{T}$ 决定时间分配,两者都可被优化;这是"能自动压缩/拉伸时间以满足动力学约束"的关键;
- **$O(M)$ 的求值与梯度**:每次迭代线性复杂度,毫秒级,可以机载在线跑;
- **与安全走廊天然契合**:每一段轨迹落在哪个凸胞是事先分配好的,路标点放在相邻凸胞的交集中;
- **与 B 样条/贝塞尔路线(Fast-Planner / EGO-Planner)对比**:B 样条以控制点为决策变量,靠**凸包性质**把动力学约束变成线性约束(简单、保守);MINCO 靠"严格经过路标点 + 惩罚函数/光滑满射"(更灵活、能直接优化时间分配,但需要非线性优化器)。两者是当前四旋翼轨迹优化的两条主流路线;
- **局限**:梯度类方法只能保证局部最优;有约束时 MINCO 只是原问题的一个**松弛**(多项式不可能精确表示所有最优解);惩罚函数要达到高精度需要较大的权重或较密的采样。

**延伸阅读**:GCOPTER(论文开源实现)、RAPTOR(ZJU-FAST-Lab 的大规模多机规划)、以及基于 MINCO 的编队/多机时空联合规划。 