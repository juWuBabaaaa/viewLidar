#view lidar

## V1
输入为目录文件，然后逐一可视化。

遇到的问题：PCL 关闭不能完全关闭窗口，导致窗口累积，窗口流畅性变差。

## V2
改变view_kitti结构，一次可视化一帧。

如果需要连续可视化，用python执行命令行的方式，连续执行上述程序。

    |-view_lidar
        |-main.py
        |-viewFrame
            |-build
            |-src
                |-main.cpp
                |-tools.h

问题1：cmake build可执行文件能够输入吗？(可以，已经验证确认。)
问题2：python怎么通过命令行关闭正在运行的程序？或者仍然使用registerMouse?


