import os
import random
'''
give lidar directory path, box directory path. 

view frame randomly. (Randomly choose one box file, then find the corresponding lidar file)
'''

ldp = "/media/wsx/Elements/kitti/training/velodyne_reduced"
bdp = "/media/wsx/Elements/kitti/training/predictions_rectified"

lfiles = os.listdir(ldp)
bfiles = os.listdir(bdp)
count = 0
while True:
    bfile = random.choice(bfiles)
    bn = bfile.strip(".txt")
    bp = os.path.join(bdp, bfile)
    lp = os.path.join(ldp, bn+".bin")

    os.system(f"./view_lidar/viewFrame/build/viewFrame {lp} {bp}")
    count += 1
    if count >= 2000:
        break

