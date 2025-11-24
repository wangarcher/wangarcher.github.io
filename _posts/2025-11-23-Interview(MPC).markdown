---
layout: post
title:  "Interview(MPC)"
subtitle: "MPC相关"
date:   2025-11-23 21:48:00
categories: [jotting, review]
---

### MPC的本质是什么？

### MPC的典型优化过程是什么？

### OSQP的典型优化过程是什么？

### OCS2的典型优化过程是什么？

1. 在第$k$次迭代中，
```
线性化 dynamics 和 constraints
二次近似 cost
得到一个 quadratic program (QP)
求解 QP 得到一个 增量方向
更新轨迹，重复迭代
```