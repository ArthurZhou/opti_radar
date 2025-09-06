# 一种基于RANSAC与Levenberg-Marquardt的3D目标高精度定位方法

## 摘要

本文提出了一种用于解决复杂环境中三维空间目标定位问题的新方法。该方法结合了**RANSAC（随机抽样一致）算法的鲁棒性**和**Levenberg-Marquardt（LM）算法的精确性**。通过多测量站点的方位数据，本系统能够有效识别并剔除由噪声产生的异常测量值，并对真实目标位置进行高精度优化。实验结果表明，该方法在处理大量噪声和异常数据时表现出优秀的稳定性和准确性。

## 引言

本项目的灵感来自 [Pixeltovoxelprojector](https://github.com/ConsistentlyInconsistentYT/Pixeltovoxelprojector) \[1\]。设想一个场景，相距不远的几台普通摄像机对着天空拍摄，通过帧差分检测画面中运动的物体，计算得出目标相对于摄像机的朝向和仰角，并提交给处理模块。

处理模块需要建立笛卡尔坐标系，将摄像机获取的数据绘制为空间直线，求解出直线的交点，找到被拍摄目标的空间坐标。为此，设计了一套包含两个核心步骤的定位流程：**基于RANSAC的异常值剔除** 和 **基于Levenberg-Marquardt的精确优化**

## 数学算法与原理

该方法核心为两种互补算法：

* **RANSAC**：随机采样一致算法，可在含有异常值的数据中找到最佳模型参数 \[2\]。
* **Levenberg-Marquardt (LM)**：非线性最小二乘优化算法，优化点到光线的距离平方和 \[3,4\]。

### 算法选择理由

相较于该项目的早期验证版本，使用RANSAC和LM替换了原本的DBSCAN和梯度下降算法，理由如下：

* **RANSAC vs DBSCAN**：RANSAC专注于拟合，通过随机采样找到最匹配的光线子集。DBSCAN属于通用聚类算法，需要额外判断簇对应目标，在多目标或噪声环境下容易出错。
* **LM vs 梯度下降**：梯度下降需要手动调学习率，且易受初值影响。LM通过阻尼因子动态调整，实现梯度下降与牛顿法自适应切换，提高收敛速度与精度。

## 目标处理流程

核心函数 `find_targets` 协调整个流程：

```markdown
原始测量数据 → 光线建模(get_line) → RANSAC筛选(ransac_fit_lines)
→ Levenberg-Marquardt优化(levenberg_marquardt_optimize) → 平均误差计算 → 输出LocatedTarget
```

* **输入**：
  * `data`：测量数据数组
  * `ransac_threshold_m`：内点阈值
  * `min_lines_per_target`：目标最少光线数
* **输出**：
  * `Vec<LocatedTarget>`：包含目标位置、光线数量及平均残差
* **流程**：
  1. 将测量数据转换为光线对象
  2. 循环识别目标
  3. 对剩余光线执行 `ransac_fit_lines`，得到候选初始位置及内点集合
  4. 使用 `levenberg_marquardt_optimize` 精确优化
  5. 计算平均误差，生成 `LocatedTarget`
  6. 标记使用光线，避免重复
  7. 若无法找到新的内点模型则结束循环

## 算法模块说明

### RANSAC 算法模块

随机迭代寻找内点最多的模型，为优化提供可靠初值 \[2\]。

```rust
pub fn ransac_fit_lines(
    all_lines: &[Line],
    ransac_iterations: usize,
    ransac_threshold: f64,
    min_lines: usize,
) -> Option<(Point3<f64>, Vec<usize>)> { ... }
```

### Levenberg-Marquardt 算法模块

迭代优化目标位置，使点到光线的距离平方和最小。通过调整阻尼因子，自动切换梯度下降与高斯-牛顿法，提高收敛速度和精度 \[3, 4\]。

```rust
pub fn levenberg_marquardt_optimize(
    lines: &[Line],
    initial_guess: Point3<f64>,
    iterations: usize,
    initial_lambda: f64,
) -> Point3<f64> { ... }
```

## Rust工程实现

核心算法完全使用内置 Rust 代码实现，只依赖 nalgebra 提供矩阵与向量运算和 rand 提供的随机数。可快速编译部署至 RISC-V/ARM 平台。

包含单元测试：

* 向量归一化验证
* 光线交点计算验证
* RANSAC鲁棒性测试
* LM精度验证

## 结论

结合 RANSAC 与 LM 算法，实现了对噪声和异常值具有高鲁棒性的三维目标定位系统。该方法能从复杂光线交汇数据中准确识别并定位多个目标，为复杂和不确定环境下的精确空间定位提供可行方案。

## 实验数据

运行 `charts.py` 可以自动生成可视化数据图表(需要先执行 `cargo build`，非Windows环境请修改python文件中的可执行文件位置)，某一次运行的结果如下：
[](./sample_result.png)

[](./sample_accuracy.png)

## 参考文献

[1] Pixeltovoxelprojector: (<https://github.com/ConsistentlyInconsistentYT/Pixeltovoxelprojector/>)

[2] Fischler, M. A., & Bolles, R. C. (1981). Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography. Communications of the ACM, 24(6), 381–395. (<https://dl.acm.org/doi/10.1145/358669.358692>)

[3] Levenberg, K. (1944). A method for the solution of certain non-linear problems in least squares. Quarterly of Applied Mathematics, 2(2), 164–168. (<https://www.jstor.org/stable/43633451>)

[4] Marquardt, D. W. (1963). An algorithm for least-squares estimation of nonlinear parameters. Journal of the Society for Industrial and Applied Mathematics, 11(2), 431–441. (<https://www.jstor.org/stable/2098941>)

[5] Szeliski, R. (2010). Computer Vision: Algorithms and Applications. Springer. 公开出版物链接: (<https://vim.ustc.edu.cn/_upload/article/files/d4/87/71e9467745a5a7b8e80e94007d1b/4cd69b21-85d3-43ba-9935-fd9ae33da82b.pdf>)

编写项目源码时，使用了Gemini 2.5 Flash辅助解释算法原理（我不会高数啊！）

***

本项目的Github地址：<https://github.com/ArthurZhou/opti_radar>
