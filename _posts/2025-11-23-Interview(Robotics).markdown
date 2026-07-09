---
layout: post
title:  "Interview(MPC)"
subtitle: "机器人学"
date:   2025-11-23 21:48:00
categories: [jotting, review]
---

# 逆运动学数值求解器详解

## 1. 函数概述

该函数实现了基于 Pinocchio 库的**数值逆运动学**，使用阻尼最小二乘法（Damped Least Squares, DLS）在给定目标位姿下迭代求解关节角 `q_ref`。

```
void MPCControllerROS::inverseKinematics(const pinocchio::SE3 &target_pose,
                                         Eigen::Matrix<double, 6, 1> &q_ref)
```

---

## 2. 求解流程

### 2.1 初始化

```
int iter = 0;
Eigen::VectorXd v(model_.nv);
Eigen::Matrix<double, 6, 1> err;
pinocchio::Data::Matrix6x J(6, model_.nv); J.setZero();
```

- `iter`：迭代计数器。
- `v`：关节速度向量（维度为 `model_.nv`）。
- `err`：6 维误差向量（平移 + 旋转，末端局部坐标系下）。
- `J`：6×n 雅可比矩阵，初始化为零。

### 2.2 迭代循环

#### 2.2.1 正向运动学更新

```
pinocchio::forwardKinematics(model_, data_, q_ref);
pinocchio::updateFramePlacements(model_, data_);
```

根据当前关节角 `q_ref` 计算所有关节及附加框架在世界坐标系下的位姿，并更新 `data_.oMf`。

#### 2.2.2 计算位姿误差

```
const pinocchio::SE3 iMd = data_.oMf[ee_frame_id_].actInv(target_pose);
err = pinocchio::log6(iMd).toVector();
```

- `iMd`：表示从**当前末端框架**到**目标框架**的相对刚体变换，**表达在末端局部坐标系**下（详见第 3 节）。
- `pinocchio::log6(iMd)` 将变换映射为李代数元素，得到 6 维螺旋向量，即局部坐标系下的位姿误差。误差为零时 `iMd = I`，`err = 0`。

#### 2.2.3 收敛检查

```
if (err.norm() < tolerance_) break;
```

若误差范数小于设定阈值，求解成功，退出循环。

#### 2.2.4 计算局部雅可比并修正

```
pinocchio::computeFrameJacobian(model_, data_, q_ref, ee_frame_id_, pinocchio::LOCAL, J);
```

获得末端在**局部坐标系**下的几何雅可比 `J`，满足：
$$ \mathbf{v}_{\text{local}} = \mathbf{J} \dot{\mathbf{q}} $$

随后引入对数映射的微分：

```
pinocchio::Data::Matrix6 Jlog;
pinocchio::Jlog6(iMd.inverse(), Jlog);
J = -Jlog * J;
```

- 误差变化率与局部空间速度的关系为：
  $$ \dot{\mathbf{e}} = -\mathbf{J}_{\log}(\mathbf{T}^{-1}) \, \mathbf{v}_{\text{local}} $$
- 因此 **误差关于关节角的雅可比** 为：
  $$ \mathbf{J}_{\text{err}} = -\mathbf{J}_{\log} \cdot \mathbf{J}_{\text{frame}} $$
- 代码通过 `J = -Jlog * J` 覆盖了原来的 `J`，此后 `J` 就代表 ∂e/∂q。

#### 2.2.5 阻尼最小二乘法求解关节速度

```
pinocchio::Data::Matrix6 JJt;
JJt.noalias() = J * J.transpose();
JJt.diagonal().array() += 1e-6;
v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
```

- 计算 6×6 矩阵 `JJt = J * J^T + λ² I`（λ² = 1e-6）。
- LDLT 分解求解 `α = (J J^T + λ² I)⁻¹ err`。
- 关节速度 `v = -J^T α`。
- 该过程等价于求解阻尼最小二乘的对偶形式（详见第 4 节）。

#### 2.2.6 更新关节角

```
q_ref = pinocchio::integrate(model_, q_ref, v * 0.1);
```

将速度乘以步长 0.1，并用流形积分更新 `q_ref`。

---

## 3. `iMd` 的含义

`iMd` 是一个 `pinocchio::SE3`，表示**当前末端执行器坐标系**到**目标坐标系**的变换，且表达在**末端本地坐标系**下。

数学定义：
$$ \text{iMd} = \mathbf{M}_{ee}^{-1} \cdot \mathbf{M}_{\text{target}} $$
其中 `M_ee` 是末端当前位姿（世界→末端），`M_target` 是目标位姿（世界→目标）。

变量命名推测：
- `i`：来自 `actInv` 中的逆
- `M`：SE3 矩阵
- `d`：desired（期望）或 delta（差异）

对该变换取对数映射得到的就是用于控制的局部螺旋误差。

---

## 4. 阻尼最小二乘法（DLS）详解

### 4.1 问题描述

希望找到关节速度 `q̇` 使误差减小：
$$ \mathbf{J} \dot{\mathbf{q}} = -\mathbf{e} $$

当 J 不可逆（冗余、欠驱动或奇异）时，需要正则化。

### 4.2 DLS 基本思想

最小化以下代价函数：
$$ \min_{\dot{\mathbf{q}}} \ \| \mathbf{J}\dot{\mathbf{q}} + \mathbf{e} \|^2 + \lambda^2 \|\dot{\mathbf{q}}\|^2 $$
其中第一项降低任务误差，第二项抑制过大关节速度。

直接解为：
$$ \dot{\mathbf{q}} = -(\mathbf{J}^T\mathbf{J} + \lambda^2 \mathbf{I})^{-1} \mathbf{J}^T \mathbf{e} $$

### 4.3 对偶形式（代码采用）

设 `q̇ = -J^T α`，代入代价函数可推导出：
$$ (\mathbf{J}\mathbf{J}^T + \lambda^2 \mathbf{I})\,\boldsymbol{\alpha} = \mathbf{e} $$
此时只需解 6×6 系统（任务空间维度），效率更高。

求解 α 后得到：
$$ \dot{\mathbf{q}} = -\mathbf{J}^T \boldsymbol{\alpha} $$

### 4.4 阻尼项 λ² = 1e-6 的作用

- 保证矩阵可逆，避免数值崩溃
- 平滑奇异构型附近的解
- 略微牺牲收敛速度以换取稳定性

---

## 5. 算法总结

1. 计算末端局部坐标系下的位姿误差 `err = log(iMd)`
2. 求误差关于关节角的雅可比 `J_err = -Jlog * J_frame`
3. 用对偶阻尼最小二乘求解关节速度 `v = -J^T (J J^T + λ² I)⁻¹ err`
4. 按固定步长积分更新关节角
5. 循环直到误差收敛或达到最大迭代次数

该方法可处理冗余、奇异及非满秩情况，适用于实时逆运动学求解。
```