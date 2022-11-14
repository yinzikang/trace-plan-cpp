# 介绍
- 目标：对机器人进行的路径规划 
- 空间： 关节空间、迪卡尔空间 
- 路径：直线：圆弧 
- 位姿插补速度曲线：
腰为s曲线的梯形，包含三个过程。
先加速达到预定速度、再匀速，最后减速。
两次速度变化的起点与终点的加速度均为0
加速过程与减速过程占全过程比重可调，最大速度可调，加速过程与减速过程对称
速度方程为ax^3,a满足-3/2*a*t^4+a*T*t^3=1，T为总时间，t为加速时间，即在时间达到T时，速度积分为1
- 其他
机器人的姿态与位置（实际走出的轨迹）是两个无关的问题，因此
例如机器人可以不旋转走一条圆弧，也可以旋转并走一条直线
两个位置可以确定一条直线中任意位置，但是无法确定一个圆
两个姿态有多种方法确定中间姿态，即多种插补方法，最常用为轴角插补

# 具体
- 迪卡尔空间直线

- 迪卡尔空间圆弧

- 迪卡尔空间点到点（关节空间规划）

- 其他
函数中各参数含义相同。与PathAlongLine注释中同步