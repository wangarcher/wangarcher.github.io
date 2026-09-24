---
layout: post
title:  "Interview-VLA"
subtitle: "询问问题"
date:   2025-11-24 09:45:00
categories: [jotting, review]
---

## VLM / Multimodal Transformer
##### 一张 RGB 图像是怎么进入 Transformer 的？

RGB 图像

  → resize/归一化        [备注：缩放到 224×224，归一化到 [-1,1]，转为 3×224×224 张量]

  → 切成 14×14 patch     [备注：patch size=14，共 16×16=256 个 patch，每个 patch 大小 14×14×3]

  → patch embedding      [备注：每个 patch 展平为 588 维，线性映射到 SigLIP 隐藏维度 1152，并加入位置编码]

  → SigLIP ViT 编码      [备注：256 个 patch token 经过 ViT，输出形状 [B, 256, 1152]]

  → 投影到语言空间       [备注：线性/MLP 投影，1152 → 2048 维，对齐 Gemma 嵌入空间]

  → 与 <image> 位置的文本 token 合并   [备注：文本中的 <image> 占位符被 256 个视觉 token 替换]

  → 进入 Gemma Transformer            [备注：多模态序列作为 inputs_embeds 输入，输出动作 token]


##### SigLIP ViT

1. LayerNorm
在进入自注意力之前，先对每个 token 的特征做 LayerNorm，稳定训练。

x_norm = LayerNorm(x)   # [B, 256, 1152]

2. 多头自注意力（Multi-Head Self-Attention）

这是 Transformer 的核心，让每个 token 都能“看到”其他所有 token，从而融合全局信息。

a) 生成 Q, K, V
对每个 token，通过三个线性层分别生成 Query、Key、Value 向量：

Q = x_norm @ W_Q   # [B, 256, 1152]

K = x_norm @ W_K   # [B, 256, 1152]

V = x_norm @ W_V   # [B, 256, 1152]

然后把维度拆成多个头，例如 num_heads = 16，每个头的维度 head_dim = 1152 / 16 = 72。

Q -> [B, 16, 256, 72]

K -> [B, 16, 256, 72]

V -> [B, 16, 256, 72]

b) 计算注意力分数

每个 token 的 Query 与所有 token 的 Key 做点积，得到注意力分数矩阵，再除以 sqrt(head_dim) 缩放，最后用 softmax 归一化成权重。

attention_scores = Q @ K^T / sqrt(72)   # [B, 16, 256, 256]

attention_weights = softmax(attention_scores, dim=-1)

这个 256×256 的矩阵表示：对于每个 token，它应该用多大的权重去关注其他 token。

c) 加权求和

用注意力权重对 Value 加权求和，得到每个 token 的更新表示：

attention_output = attention_weights @ V   # [B, 16, 256, 72]

然后把多个头拼接回原来的维度：

attention_output -> [B, 256, 1152]
d) 残差连接

将自注意力的输出加到原始输入上（残差连接），帮助梯度流动：

x = x + attention_output   # [B, 256, 1152]

3. 前馈网络（MLP）

自注意力之后，再经过一个两层的全连接网络，通常带 GELU 激活函数，中间维度一般扩大 4 倍（例如 1152 → 4304 → 1152）。

x_norm2 = LayerNorm(x)          # [B, 256, 1152]

mlp_output = Linear2(GELU(Linear1(x_norm2)))   # [B, 256, 1152]

然后同样使用残差连接：

x = x + mlp_output

4. 重复 L 层

以上步骤重复 L 次（例如 27 次），每一层的权重不同。经过所有层后，输出仍然是：

[B, 256, 1152]

但此时每个 token 已经融合了整个图像的上下文信息，而不再只是表示单个 patch 的局部特征。


##### 为什么不能直接把 CLIP feature 喂给 LLM？

主要原因是 维度不匹配、模态嵌入空间不对齐，以及 特征形式不适合序列输入。

1. CLIP/SigLIP 视觉编码器的输出维度通常与 LLM 的隐藏维度不同。

例如 OpenPI 中使用的 SigLIP 隐藏维度是 1152，而语言主干 Gemma 的隐藏维度是 2048。

解决方法：使用一个线性层或 MLP（即 projector）将视觉特征从 1152 维映射到 2048 维，使其与 LLM 的嵌入维度一致。


2. 模态嵌入空间不对齐

CLIP 的视觉编码器通过对比学习，将图像和文本映射到同一个语义空间（用于图文匹配），但这个空间是 CLIP 自己定义的，不一定适合 LLM 的推理任务。

LLM 的文本嵌入是在大规模文本语料上训练出来的，它对 token 的表示分布、几何关系有自己的特性。

3. 特征形式不适合直接作为序列输入

LLM 是自回归序列模型，其输入是一个 token 序列。

CLIP 通常输出两种特征：

全局图像特征（如 [CLS] token 或池化后的向量）：只是一个向量，无法构成序列，会丢失空间细节，不利于需要细粒度视觉理解的任务（如机器人动作生成）。

Patch tokens（如果不使用池化）：可以构成序列，但其数量和维度不一定与 LLM 兼容。例如 SigLIP 输出 256 个 patch token，每个 1152 维；LLM 期望 token 嵌入维度是 2048 维，且文本 token 通常有特定的起始/结束符号。

4. 训练稳定性与效率

如果不使用投影层，直接在训练中让视觉编码器和 LLM 的参数共同适应，可能会导致：

灾难性遗忘：LLM 在微调时容易丢失原有的语言能力。

训练不稳定：两个模态的梯度尺度差异大，难以收敛。

计算开销大：需要同时更新视觉编码器和 LLM 的大量参数。


##### MLP Projector、Q-Former 和 Cross-Attention
在视觉-语言模型中，把视觉特征送入大语言模型（LLM）主要有三种设计：**MLP Projector、Q-Former 和 Cross-Attention**。它们的核心区别在于视觉信息以什么形式、在什么位置进入语言模型。

**MLP Projector** 是最直接的方式：把视觉编码器（如 SigLIP）输出的 patch token 通过一个简单的线性层或 MLP 映射到 LLM 的嵌入维度，然后直接拼接到文本 token 序列中，让 LLM 像处理普通文本一样处理这些视觉 token。它的优点是实现简单、训练稳定，而且保留了完整的空间细节（比如 256 个 patch token 全部保留），适合机器人操作这类需要精确空间理解的任务，OpenPI 和 LLaVA 用的就是这种方式。缺点是视觉 token 数量较多，会增加 LLM 的序列长度和计算量。

**Q-Former** 则引入了一个额外的 Transformer 模块，里面有一组可学习的 query token（通常 32 个）。这些 query token 通过 cross-attention 从图像特征中主动提取与任务相关的信息，最终输出固定数量的视觉 token（比如 32 个），再投影后拼接到文本序列中。它的好处是大幅压缩了视觉 token 数量，降低 LLM 的计算负担，而且 Q-Former 可以在预训练阶段单独训练，灵活适配不同的 LLM。缺点是信息瓶颈可能丢失一些细节，对需要精确空间位置的任务不够友好，代表模型是 BLIP-2 和 InstructBLIP。

**Cross-Attention** 的思路完全不同：它不把视觉 token 放进 LLM 的输入序列，而是在 LLM 的某些层中插入额外的 cross-attention 模块，让文本 token 作为 query 去动态查询视觉特征（作为 key/value）。这样视觉特征不占用输入长度，LLM 可以在生成过程中按需关注图像信息。优点是理论上是最高效、最灵活的，尤其适合处理高分辨率或大量视觉特征。缺点是必须修改 LLM 架构，训练通常不稳定，而且往往需要冻结 LLM 主干，实现复杂。Flamingo 系列和 CogVLM 采用了类似思路。

总结来说，**MLP Projector 简单直接、保留细节**，适合空间敏感任务；**Q-Former 主动压缩信息、减少 token 数**，适合计算敏感但细节要求不高的任务；**Cross-Attention 不占序列、按需查询**，理论上最灵活但实现和训练最难。OpenPI 选择 MLP Projector，是因为机器人控制需要保留完整的视觉空间信息，同时训练流程也要足够简单稳定。


##### Action chunking 
核心思想是让模型一次预测未来连续多个动作（例如 k 个时间步），而不是仅预测当前一个动作。这确实能提升动作的时序一致性、减少短视误差，并在一定程度上对抗累积误差，但它也带来了一些新的问题。

首先，**反应延迟会增大**。如果模型每预测一个 chunk 后就要完整执行这 k 个动作，那么在这 k 步执行期间，模型无法根据最新的环境观测做出调整。对于需要快速响应的动态任务（如移动物体抓取、避障），这种延迟可能导致失败。即使采用“执行一步再重新预测”的策略，虽然延迟减小了，但计算开销会大幅上升，削弱了 chunking 带来的效率优势。

其次，**误差累积与状态偏移**。预测的 chunk 越长，后面几步动作所基于的状态就与真实环境偏离越远，因为执行过程中环境已经发生了变化。模型在预测时假设环境静态，但实际并非如此，这会导致 chunk 后半部分的动作准确率下降，甚至产生危险动作。

第三，**可能过度平滑动作细节**。为了生成一个连贯的 chunk，模型往往会倾向于输出平滑的动作序列，这在高精度操作中可能丢失一些高频、精细的调整能力，尤其是在需要快速微调的接触式操作中，平滑化反而有害。

第四，**训练和推理开销显著增加**。输出维度从 `action_dim` 变为 `k × action_dim`，模型输出头需要更大的容量，训练时需要更多的数据来约束这些额外自由度，推理时也需要更多计算，对实时性造成压力。

最后，**重规划时的动作不连续**。如果模型在 chunk 执行到一半时被新观测触发重新规划，新旧 chunk 之间的动作可能存在跳变，导致机器人抖动。如果不做平滑处理，会影响执行稳定性和安全性。

因此，实际系统中需要在 chunk 长度和执行策略之间做权衡：较短的 chunk 能提高反应速度但损失一些时序一致性；较长的 chunk 能提升效率但引入延迟和误差。通常会结合重规划频率、动作平滑等技术来缓解这些问题。

# π₀-FAST 中的动作表示与生成机制（核心理解）

## 1. 自回归离散（Autoregressive Discrete Modeling）

自回归（Autoregressive, AR）建模将序列分解为条件概率连乘：

$$
p(x_{1:T}) = \prod_{t=1}^{T} p(x_t \mid x_{<t})
$$

在 π₀-FAST（Pi-Zero Fast）中：

- 输出不是连续动作，而是**离散 token（标记）**
- 每一步预测下一个 token（类似语言模型）

关键特点：

- 顺序生成（sequential generation）
- 使用 Transformer（Transformer 神经网络）建模 token 分布
- 将控制问题转化为“序列建模问题”

---

## 2. FAST：频域动作标记化（Frequency-space Action Sequence Tokenization）

FAST（Frequency-space Action Sequence Tokenization）核心思想：

> 将时间域动作序列映射到频域，再进行离散化（tokenization）

整体流程：

$$
\text{action trajectory } x \;\xrightarrow{\text{DCT}}\; C \;\xrightarrow{\text{quantization}}\; \text{tokens}
$$

其中：

- DCT（Discrete Cosine Transform，离散余弦变换）将动作转换到频域
- token 表示**频域系数的离散索引**
- Transformer 学习的是 token 序列分布

---

## 3. DCT / IDCT 不是学习得到的

一个关键点：

> DCT（离散余弦变换）和 IDCT（Inverse Discrete Cosine Transform，逆离散余弦变换）是固定线性变换，而非神经网络学习模块

形式上：

$$
C = D \cdot x
\quad,\quad
x = D^{-1} \cdot C
$$

其中：

- $D$ 是固定的正交变换矩阵
- $D^{-1} = D^\top$

因此：

- 无需训练 encoder / decoder
- 无额外 reconstruction loss
- 无信息瓶颈（理想情况下）

---

## 4. 从 token 到动作序列的恢复

推理阶段流程：

$$
\text{tokens} \rightarrow \text{coefficients } C \rightarrow \text{IDCT} \rightarrow x_{1:T}
$$

具体步骤：

1. token → embedding（嵌入向量）
2. embedding → 频域系数（coefficient）
3. IDCT（逆离散余弦变换）还原时间序列

关键理解：

> token 并不表示动作本身，而是“动作轨迹的频域表示”

---

## 5. 为什么在频域建模？

### （1）低频主导（Low-frequency dominance）

机器人动作通常是平滑的：

- 主要信息集中在低频
- 高频仅用于细节修正

---

### （2）更适合离散化（Tokenization-friendly）

相比时间域：

- 频域更稀疏
- 分布更规则
- 更容易量化为 token

---

### （3）长时依赖更容易建模

在时间域：

- 长序列难建模
- 误差易累积

在频域：

- 每个系数影响全局轨迹
- 更适合 Transformer 学习全局结构

---

## 6. 与其他方法对比

| 方法 | 表示空间 | 生成方式 | 特点 |
|------|----------|----------|------|
| 自回归离散（AR discrete） | token | 逐步生成 | 快，但有离散误差 |
| Diffusion policy（扩散策略） | 连续 | 迭代去噪 | 稳定但慢 |
| Flow matching（流匹配） | 向量场 | 连续变换 | 全局一致 |
| π₀-FAST | 频域 token | 自回归 | 快 + 保留轨迹结构 |

---

## 7. 核心总结

> π₀-FAST 的本质是：  
> **使用固定频域基（DCT）将连续动作映射到结构化空间，再通过自回归 Transformer 建模其离散表示，最后通过 IDCT 还原为连续控制轨迹。**

可以理解为：

- 不是逐步生成动作（time-domain control）
- 而是生成“动作的频谱描述”（frequency-domain representation）

类似：

> “先生成乐谱，再演奏动作”