
3220104688 杨佳昕

# 一、最近点搜索：

利用KDTree来搜索离输入点云src.point\[i]相近的目标点云dst.point\[i]:

```python
def nearest_search(src, dst):
	kdtree = KDTree(dst)
	_, index = tree.query(src, k=1)
	return index
```

上面的代码是从src到dst的单向搜索。考虑到不同的*src.point\[i]* 可能会找到相同的*dst.point\[c]*，因此引入双向搜索，即先让src搜索dst，再让dst搜索src，只有双方搜索出来的点对相同的时候才保留点以及匹配点。

另外，为了确保不会搜索到过远的点导致出现较大误差，在nearest_search()中，设定了最大距离*max_distance*，凡是搜索时两点距离超过了*max_distance*均会被忽略。

以上措施使误差缩减了接近50%，达到作业中的相对误差要求。

# 二、SVD：

以下是SVD算法的理论推导：

对于src点云$P = \left[\begin{matrix}p_1&p_2&...&p_n\end{matrix}\right]$和dst点云$Q = \left[\begin{matrix}q_1&q_2&...&q_n\end{matrix}\right]$，有：

$$ q_i = Rp_i + t $$

需要实现的函数是：

$$ \underset{R, t}{argmin}\frac{1}{2}\sum_{i=1}^n {||q_i-Rp_i-t||}^2 \triangleq \underset{R, t}{argmin} DST$$

利用SVD求解该函数：

定义点云质心$\upsilon_p = \frac{1}{n} \sum_{i=1}^n p_i$，$\upsilon_q=\frac{1}{n} \sum_{i=1}^n q_i$，则：

$$
DST = \frac{1}{2}\sum_{i=1}^n {||q_i-Rp_i-t||}^2
$$

$$
DST= \frac{1}{2} \sum_{i=1}^n ({||q_i-\upsilon_q-R(p_i-\upsilon_p||}^2 + {||\upsilon_q-R\upsilon_p-t||}^2 + 2(q_i-\upsilon_q - R(p_i - \upsilon_p))^T (\upsilon_q - R\upsilon_p - t))
$$
$$
DST = \frac{1}{2} \sum_{i=1}^n ({||q_i-\upsilon_q-R(p_i-\upsilon_p||}^2 + {||\upsilon_q-R\upsilon_p-t||}^2
$$

优化问题变为：

$$ R^* = \underset{R}{argmin} \frac{1}{2} \sum_{i=1}^{n} {||q_i'-Rp_i'||}^2 \tag{1}$$
$$ t^* = \upsilon_q - R\upsilon_p $$

对(1)，有：

$$
R^* = \underset{R}{argmin} \frac{1}{2} \sum_{i=1}^n ({q_i'}^T q_i + {p_i'}^T R^T R p_i' - 2{q_i'}^T R p_i') \\ = \underset{R}{argmin} \sum_{i=1}^n -{q_i'}^T R p_i'
$$

令$W = \sum_{i=1}^n q_i' {p_i'}^T$，当$W$正交时，通过SVD分解有：$W = U\Sigma V^T$

且：
$$ R^* = UV^T $$
$$ t^* = \upsilon_q - R\upsilon_p $$

在代码中，为了让计算更加方便，在定义了点云质心以后进行了去中心化，即所有点坐标同时减去中心点坐标，变成了点云相对于点云质心的相对坐标，以此进行下一步计算。

由于在SVD算法中并没有严格限制W正交，因此当$det(W)<0$时，要强制将W变为正交阵。具体做法是将W的最后一列都乘以-1，也就相当于把$R$矩阵的最后一列取反，即：

```python
if np.linalg.det(R) < 0:
	R[:, -1] = -R[:, -1]
```

由此可求出旋转矩阵$R$和平移矩阵$t$，可计算齐次变换矩阵$T$.

# 三、ICP算法实现：

具体操作和前面类似，即先通过nearest_search()函数，利用双向搜索搜索到所有符合条件的\[src, dst]点对，再用svd计算其旋转矩阵$R$, 平移矩阵$t$以及齐次变换矩阵$T$。

在代码中，设定了阈值*threshold*，对于把输入点云进行了旋转与平移变换的点云*current_src*，如果与目标点云*dst*的欧几里得距离小于*threshold*，则停止旋转与平移变换，认为当前点云为最终结果。

# 四、相对误差计算：

相对误差等于绝对误差除以路径长度，即：

```python
relative_error = error / length
```

绝对误差计算只需要计算每个最终点云与目标点云的xyz坐标的差，以下是路径长度的计算过程：

路径长度指的是dst点云每两个相邻点之间的欧几里得距离和，为了求两相邻点之间距离，先算出两相邻点的向量，再计算其范数：

```python
diff = np.diff(dst_points, axis=0)
traj_length = np.linalg.norm(diff, axis=1)
```

将各个*traj_length*加和即可得到路径长度。

# 五、实验结果：

**导入点云：**
```python
src = o3d.io.read_point_cloud("./ply_files/0.ply")
dst = o3d.io.read_point_cloud("./ply_files/1.ply")
```
*Note: 如路径改变或想修改ply点云序号，请修改本部分代码。*

main()函数中导入了通过MATLAB中pcregistericp()计算得到的10组真值*true_value*，通过计算得出每对点云转换的$R, t$以及$T$矩阵，并得到相对误差以及平移矩阵$t$与*true value*的欧几里得距离差，作为用来进行作业验证的误差值*true_error*.

后面的代码为可视化部分。

**实验结果如下**（篇幅有限，仅以0.ply->1.ply为示例，其余点云对请自行修改代码进行试验）：

```text
Relative error: 2.3403599508997335
True error: 1.074923877047211

Rotation:
 [[ 0.9997321  -0.02314574  0.        ]
 [ 0.02314574  0.9997321   0.        ]
 [ 0.          0.          1.        ]]

Translation:
 [-0.04650576  0.01898999  0.        ]

Homogeneous:
 [[ 0.9997321  -0.02314574  0.         -0.04650576]
 [ 0.02314574  0.9997321   0.          0.01898999]
 [ 0.          0.          1.          0.        ]
 [ 0.          0.          0.          1.        ]]
```

**点云可视化**（可以以中心点为旋转中心自由旋转）见附件result0to1.png.

# 六、遇到的问题：

1. 由于在nearest_search()函数中进行了双向匹配，虽然会让准确率明显增加，但会导致可用的点减少。如果点提供的点云ply文件点数过少，有可能会导致无法匹配。

	**曾考虑的改进方法：**
	
	不使用双向匹配方法，由于src的点云数量一定小于dst的点云数量，因此可以当src.point\[i]匹配到一个dst.point\[i]以后，其余src点不再匹配这个dst点。
	
	但是这样做的话，通过实验验证会引起更大的误差，因为确实存在有几个src点本身就应该匹配到同一个dst点的可能，在匹配过程中，还是不能硬性的直接进行匹配。
	

2. 在nearest_search()中，采用KDTree对输入点云和目标点云进行匹配。但如果要满足作业中少利用库函数的要求，可以直接暴力求解，即逐个点遍历并求解欧几里得距离：

```python
def nearest_search(src, dst):
	index = []
	for _, src_point in enumerate(src.points):
		min_dist = float('inf')
		min_index = -1
		for i, dst_point in enumerate(dst.points):
			dist = np.linalg.norm(src_point - dst_point)
			if dist < min_dist:
				min_dist = dist
				min_index = i
		index.append(min_index)
	return np.array(index)
```

这种方法可以不利用库函数计算邻近点，但是会导致算力和误差的急剧增加。由于作业中的ply文件中点数不是特别多，因此算力问题没有出现，但误差有了显著的增加。如果ply文件当中的点云数量特别多，甚至到了实际应用的、点云数量巨大的情况，这种方法明显是不可取的。

3. 在main()函数中，为了在保留点云结构的情况下，减少点的数量，提升算力，曾使用过体素下采样：

```python
src = o3d.io.read_point_cloud("./ply_files/0.ply")
dst = o3d.io.read_point_cloud("./ply_files/1.ply")

voxel_size = 0.1
src = src.voxel_down_sample(voxel_size)
dst = dst.voxel_down_sample(voxel_size)

src_points = np.array(src.points)
dst_points = np.array(dst.points)
```

但由于这种采样方法类似于A\*划定方形之后进行采样的方法，因此会有几个点划分到一个采样方块内并合并成一个点的可能，会导致src点云数量和dst点云数量不同，从而无法匹配。因此，不得不强制将src和dst的点云数量调整成相同：

```python
if len(src_points) > len(dst_points):
	src_points = src_points[:len(dst_points)]
elif len(dst_points) > len(src_points):
	dst_points = dst_points[:len(src_points)]
```

但这种一刀切的方法会显著增加误差，因此运用体素内采样也不可行。

4. 可视化部分中，曾使用过创建坐标轴及坐标的代码：

```python
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
```

但是输出会有警告（甚至现在也有可能有）：

```text
[Open3D WARNING] [ViewControl] SetViewPoint() failed because window height and width are not set.
```

这是由于输出界面的长度和宽度没有确定，导致坐标轴无法输出。如果要确定长度和宽度，就要创建可视化窗口：

```python
vis = o3d.visualization.Visualizer()
vis.create_window(width = 1920, height = 1080)
```

但经过试验，加入可视化窗口以后确实可以输出一个非常难看的坐标轴，但没有输出坐标。因此，最终放弃了这种方法，只保留了点云的形态图像，但没有输出坐标值。

---
<b><font color=YellowGreen face=Consolas>Intelligent Mobile Technology - ICP</font></b>
<b><font color=YellowGreen face=Consolas>JiaXin Yang</font></b>
<b><font color=YellowGreen face=Consolas>6 Apr, 2025</font></b>


