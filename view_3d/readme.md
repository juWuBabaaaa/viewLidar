这个库和view_nuscenes很像. 
This package merges *view_box* and *view_nuscenes* packages. 

# 功能
## 基本设定
* 给出点云和标注（预测）的**目录**，可视化并且可以手动调整到下一帧. 
* 标注文件命名和数据文件相同，后缀不同。
* 坐标轴颜色x, y, z(r, g, b)
* kitti数据中的label不能直接用于lidar可视化，需要用.pkl中的数据。最好将lidar的标注直接存在文件夹中。