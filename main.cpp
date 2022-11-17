#include "include/jk5_stick.h"
#include "iostream"

using namespace std;

int main() {
    Robot rbt;
    // 起点与、中点与终点
    KDL::Vector init_xpos = KDL::Vector(0.5, 0.1, 0.4);
    KDL::Rotation init_xmat = KDL::Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1);
    KDL::Frame init_frame = KDL::Frame(init_xmat, init_xpos);
    KDL::Vector mid_xpos = KDL::Vector(0.3, 0.1, 0.4);
    KDL::Rotation mid_xmat = KDL::Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1);
    KDL::Frame mid_frame = KDL::Frame(mid_xmat, mid_xpos);
    KDL::Vector end_xpos = KDL::Vector(0.4, 0.1, 0.3);
    KDL::Rotation end_xmat = KDL::Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1);
    KDL::Frame end_frame = KDL::Frame(end_xmat, end_xpos);
    cout << "desired init frame" << endl;
    rbt.ShowXPos(init_frame);
    cout << "desired end frame" << endl;
    rbt.ShowXPos(end_frame);
    cout << endl;

    // 路径相关
    double time = 3.0;
    int dot_num = 300;
    double vel_raising_percentage = 0.05;
    double vel_max = 300.;
    double **result_list;

    // 选择路径规划模式
    int mode = 2;
    if (mode == 1)
        result_list = rbt.PathToPoint(init_frame, end_frame,
                                      time, dot_num,
                                      vel_raising_percentage, vel_max, true,"point");
    else if (mode == 2)
        result_list = rbt.PathAlongCircle(init_frame, end_frame, mid_frame,
                                          time, dot_num,
                                          vel_raising_percentage, vel_max, true,"circle");
    else
        result_list = rbt.PathAlongLine(init_frame, end_frame,
                                        time, dot_num,
                                        vel_raising_percentage, vel_max, true,"line");


    return 0;
}