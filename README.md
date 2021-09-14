# 6-DOF-ConvGPD

## 基于未知物体三维点云特征的机器人六自由度抓取
针对非结构化环境中任意位姿的未知物体, 提出了一种基于点云特征的机器人六自由度抓取位姿检测方
法, 以解决直接从点云中获取目标抓取位姿的难题. 首先, 根据点云的基本几何信息生成抓取候选, 并通过力平衡等
方法优化这些候选; 然后, 利用可直接处理点云的卷积神经网络ConvPoint评估样本, 得分最高的抓取将被执行, 其
中抓取位姿采样和评估网络都是以原始点云作为输入; 最后, 利用仿真和实际抓取实验进行测试. 结果表明, 该方法在常用对象上实现了88.33%的抓取成功率, 并可以有效地拓展到抓取其他形状的未知物体.

## 示意图
- ![流程](https://github.com/quxiaochang/6-DOF-ConvGPD/blob/master/pictures/%E6%8A%93%E5%8F%96%E6%A3%80%E6%B5%8B%E7%AE%97%E6%B3%95%E6%B5%81%E7%A8%8B.jpg )
- ![单目标抓取](https://github.com/quxiaochang/6-DOF-ConvGPD/blob/master/pictures/%E5%8D%95%E7%89%A9%E4%BD%93%E6%8A%93%E5%8F%96%E5%AE%9E%E9%AA%8C.jpg )
## 依赖
- PCL 1.8 或以上
- Libtorch(与安装的Pytorch版本保持一致)

## 致谢
- [gpg](https://github.com/atenpas/gpg)
- [gpd](https://github.com/atenpas/gpd)
- [PointNetGPD](https://github.com/lianghongzhuo/PointNetGPD)
- [6dof-graspne](https://github.com/NVlabs/6dof-graspnet)
- [convPoint](https://github.com/aboulch/ConvPoint)
- [pointnet](https://github.com/fxia22/pointnet.pytorch)
