//
// Created by yzk on 11/8/22.
//

#include "sys/stat.h"
#include "unistd.h"
#include "../include/jk5_stick.h"
#include "iostream"
#include "fstream"

Robot::Robot() {
    this->kdl_model.addSegment(KDL::Segment("base_link", KDL::Joint("base_joint", KDL::Joint::Fixed),
                                            KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.069))));
    this->kdl_model.addSegment(KDL::Segment("link1", KDL::Joint("joint1", KDL::Joint::RotZ),
                                            KDL::Frame(KDL::Rotation::RotX(KDL::PI / 2), KDL::Vector(0, 0, 0.073))));
    this->kdl_model.addSegment(KDL::Segment("link2", KDL::Joint("joint2", KDL::Joint::RotZ),
                                            KDL::Frame(KDL::Rotation::RotZ(KDL::PI / 2), KDL::Vector(0, 0.425, 0))));
    this->kdl_model.addSegment(KDL::Segment("link3", KDL::Joint("joint3", KDL::Joint::RotZ),
                                            KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.395, 0, 0))));
    this->kdl_model.addSegment(KDL::Segment("link4", KDL::Joint("joint4", KDL::Joint::RotZ),
                                            KDL::Frame(KDL::Rotation::Quaternion(-0.5, 0.5, -0.5, 0.5),
                                                       KDL::Vector(0, 0, 0.1135))));
    this->kdl_model.addSegment(KDL::Segment("link5", KDL::Joint("joint5", KDL::Joint::RotZ),
                                            KDL::Frame(KDL::Rotation::RotX(KDL::PI / 2), KDL::Vector(0, 0, 0.1015))));
    this->kdl_model.addSegment(KDL::Segment("link6", KDL::Joint("joint6", KDL::Joint::RotZ),
                                            KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.094))));
    this->kdl_model.addSegment(KDL::Segment("end-effector", KDL::Joint("ee_joint", KDL::Joint::Fixed),
                                            KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.169))));

    this->joint_number = this->kdl_model.getNrOfJoints();
}

// 行为各时刻，共dot_num行；列为到达该点的时间与各关节的角度，共1+joint_number+3+4列
double **Robot::CreateResultList(int dot_num) {
    auto **result_list = new double *[dot_num];
    for (int dot_idx = 0; dot_idx < dot_num; dot_idx++) {
        result_list[dot_idx] = new double[1 + this->joint_number + 3 + 4]{0.0};//时间+各关节位置+迪卡尔位置+四元数
    }

    return result_list;
}

/*!
 * 迪卡尔空间下直线路径规划
 * 中间位置通过直线插值确定，中间姿态通过Slerp插值确定
 * 具体位姿由梯形的速度曲线确定，两条斜边为完全相同的三次样条曲线，保证起始与结束的速度加速度为0
 * 速度曲线积分得到的面积为1，当前时刻位姿由0到该时刻积分面积确定，每个方向上的角速度与线速度同步变化，即同时达到最大，同时开始减速
 * @param init_frame 初始位姿
 * @param end_frame 结束位姿
 * @param time 运动总时间，秒
 * @param dot_num 插值点个数
 * @param velocity_raising_percentage 加速时间占运行时间比例
 * @param velocity_limit 线速度的最大速度限制，如果加速时间占比不合理将导致最大速度超过速度限制，米每秒
 * @param save_file 保存文件标志位
 * @param file_name 保存文件路径名
 */
double **Robot::PathAlongLine(const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time,
                              int dot_num, double velocity_raising_percentage, double velocity_limit, bool save_file,
                              std::string file_name) {
    // 运动学求解器
    KDL::ChainIkSolverPos_LMA ikine_solver = KDL::ChainIkSolverPos_LMA(kdl_model, 1E-5, 20000);

    // 利用运动的距离、时间，求出速度的参数以及相关参数
    double **result_list = CreateResultList(dot_num);
    double delta_time = time / dot_num;
    double velocity_raising_time = velocity_raising_percentage * time; //加速所用时间
    double curve_para = 1. / (-3. / 2. * pow(velocity_raising_time, 4) + time * pow(velocity_raising_time, 3)); //样条曲线参数
    double curve_max = curve_para * pow(velocity_raising_time, 3); //速度梯形的高
    std::cout << "curve: para " << curve_para << " max: " << curve_max << std::endl;

    // 线速度，直线插值
    KDL::Vector pos_error = end_frame.p - init_frame.p;
    KDL::Vector velocity_v_max = pos_error * curve_max; //xyz三个方向最大线速度
    std::cout << "max speed: " << velocity_v_max.x() << ' ' << velocity_v_max.y() << ' ' << velocity_v_max.z() << ' '
              << std::endl;
    if (abs(velocity_v_max.x()) > velocity_limit ||
        abs(velocity_v_max.y()) > velocity_limit ||
        abs(velocity_v_max.z()) > velocity_limit) {
        std::cout << "线速度超过最大速度限制" << std::endl;
    }

    // 角速度，Slerp方法
    double w1, x1, y1, z1, w2, x2, y2, z2;
    init_frame.M.GetQuaternion(x1, y1, z1, w1);
    end_frame.M.GetQuaternion(x2, y2, z2, w2);
    double cos_rot_error = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2;
    if (cos_rot_error < 0.0f) { // 四元数点积的结果是负值（夹角大于90°）插值会走远路，该操作保证旋转为最短路径
        x2 = -x2;
        y2 = -y2;
        z2 = -z2;
        w2 = -w2;
        cos_rot_error = -cos_rot_error;
    }
    double sin_rot_error = sqrt(1. - cos_rot_error * cos_rot_error);
    double rot_error = atan2(sin_rot_error, cos_rot_error); // 返回以弧度为单位的角度，范围为[-pi,+pi]
    double velocity_w_para1, velocity_w_para2;
    std::cout << "rot_error(degree): " << rot_error * 180. / M_PI << std::endl;

    // 轨迹上每一个时刻，及到达的关节位置
    int dot_idx;
    double current_time;
    double percent; //当前速度积分值（0-1）
    KDL::Frame current_frame_desire; //当前期望坐标系
    KDL::JntArray current_qpos = KDL::JntArray(joint_number); //current_frame_desire求解得到逆解current_qpos
    KDL::JntArray last_qpos = KDL::JntArray(joint_number); // 上一次的逆解角度，用于逆动力学方便求解
    std::cout << std::endl;

    // 起点
    std::cout << "######## start ########" << std::endl;
    dot_idx = 0;
    current_time = dot_idx * delta_time;
    percent = 0.0;
    std::cout << 100 * percent << " % accomplished" << std::endl;
    current_frame_desire = init_frame; //当前期望坐标系
    ikine_solver.CartToJnt(KDL::JntArray(joint_number), init_frame, current_qpos);
    SaveToArray(current_time, current_qpos, result_list[dot_idx]);
    last_qpos = current_qpos; // 上一次的逆解角度

    //加速
    std::cout << "######## speed up ########" << std::endl;
    for (; dot_idx * delta_time < velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4. * pow(current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        current_frame_desire.p = init_frame.p + pos_error * percent;
        if (cos_rot_error > 0.9995) {
            velocity_w_para1 = 1. - percent;
            velocity_w_para2 = percent;
        } else {
            velocity_w_para1 = sin((1. - percent) * rot_error) / sin_rot_error;
            velocity_w_para2 = sin(percent * rot_error) / sin_rot_error;
        }
        current_frame_desire.M = KDL::Rotation::Quaternion(velocity_w_para1 * x1 + velocity_w_para2 * x2,
                                                           velocity_w_para1 * y1 + velocity_w_para2 * y2,
                                                           velocity_w_para1 * z1 + velocity_w_para2 * z2,
                                                           velocity_w_para1 * w1 + velocity_w_para2 * w2);
        ikine_solver.CartToJnt(last_qpos, current_frame_desire, current_qpos);
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
        last_qpos = current_qpos;
    }
    //匀速
    std::cout << "######## stable ########" << std::endl;
    for (; dot_idx * delta_time < time - velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4 * pow(velocity_raising_time, 4) + curve_max * (current_time - velocity_raising_time);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        current_frame_desire.p = init_frame.p + pos_error * percent;
        if (cos_rot_error > 0.9995) {
            velocity_w_para1 = 1. - percent;
            velocity_w_para2 = percent;
        } else {
            velocity_w_para1 = sin((1. - percent) * rot_error) / sin_rot_error;
            velocity_w_para2 = sin(percent * rot_error) / sin_rot_error;
        }
        current_frame_desire.M = KDL::Rotation::Quaternion(velocity_w_para1 * x1 + velocity_w_para2 * x2,
                                                           velocity_w_para1 * y1 + velocity_w_para2 * y2,
                                                           velocity_w_para1 * z1 + velocity_w_para2 * z2,
                                                           velocity_w_para1 * w1 + velocity_w_para2 * w2);
        ikine_solver.CartToJnt(last_qpos, current_frame_desire, current_qpos);
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
        last_qpos = current_qpos;
    }
    //减速
    std::cout << "######## speed down ########" << std::endl;
    for (; dot_idx * delta_time < time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = 2 * curve_para / 4 * pow(velocity_raising_time, 4) +
                  curve_max * (time - 2 * velocity_raising_time) -
                  curve_para / 4 * pow(time - current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        current_frame_desire.p = init_frame.p + pos_error * percent;
        if (cos_rot_error > 0.9995) {
            velocity_w_para1 = 1. - percent;
            velocity_w_para2 = percent;
        } else {
            velocity_w_para1 = sin((1. - percent) * rot_error) / sin_rot_error;
            velocity_w_para2 = sin(percent * rot_error) / sin_rot_error;
        }
        current_frame_desire.M = KDL::Rotation::Quaternion(velocity_w_para1 * x1 + velocity_w_para2 * x2,
                                                           velocity_w_para1 * y1 + velocity_w_para2 * y2,
                                                           velocity_w_para1 * z1 + velocity_w_para2 * z2,
                                                           velocity_w_para1 * w1 + velocity_w_para2 * w2);
        ikine_solver.CartToJnt(last_qpos, current_frame_desire, current_qpos);
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
        last_qpos = current_qpos;
    }
    std::cout << "end" << std::endl;

    if (save_file) {
        SaveToFile(result_list, init_frame, end_frame, time, dot_num, velocity_raising_percentage, velocity_limit,
                   "Line", file_name);
    }

    return result_list;
}

/*!
 *
 * @param init_frame 初始位姿
 * @param end_frame 结束位姿
 * @param mid_frame 中间位姿，只有位置被用到，用于确定圆弧轨迹的圆心与半径
 * @param time 运动总时间，秒
 * @param dot_num 插值点个数
 * @param velocity_raising_percentage 加速时间占运行时间比例
 * @param velocity_limit 线速度的最大速度限制，如果加速时间占比不合理将导致最大速度超过速度限制，米每秒
 * @param save_file 保存文件标志位
 * @param file_name 保存文件路径名
 * @return 插值结果数组首地址
 */
double **Robot::PathAlongCircle(const KDL::Frame &init_frame, const KDL::Frame &end_frame, const KDL::Frame &mid_frame,
                                double time, int dot_num,
                                double velocity_raising_percentage, double velocity_limit,
                                bool save_file, std::string file_name) {
    // 运动学求解器
    KDL::ChainIkSolverPos_LMA ikine_solver = KDL::ChainIkSolverPos_LMA(kdl_model, 1E-5, 20000);

    // 利用运动的距离、时间，求出速度的参数以及相关参数
    double **result_list = CreateResultList(dot_num);
    double delta_time = time / dot_num;
    double velocity_raising_time = velocity_raising_percentage * time; //加速所用时间
    double curve_para = 1. / (-3. / 2. * pow(velocity_raising_time, 4) + time * pow(velocity_raising_time, 3)); //样条曲线参数
    double curve_max = curve_para * pow(velocity_raising_time, 3); //速度梯形的高
    std::cout << "curve: para " << curve_para << " max: " << curve_max << std::endl;

    // 求旋转平面与法向量
    KDL::Vector P1 = mid_frame.p - init_frame.p; //轨迹前半段
    KDL::Vector P2 = end_frame.p - mid_frame.p; //轨迹后半段
    KDL::Vector direction = P1 * P2; //轨迹前半段插乘后半段得到旋转平面法向量，并利用右手定则确定旋转方向
    // 创建新坐标系(U,V,W,mid_frame)，即该坐标系原点位于中间点
    KDL::Vector U = -P1 / P1.Norm(); //U方向单位向量，方向为mid到init
    KDL::Vector W = direction / direction.Norm(); //W方向单位向量
    KDL::Vector V = W * U; //V方向单位向量，方向为mid到end
    KDL::Rotation R = KDL::Rotation(U, V, W); // 旋转后坐标系的旋转矩阵
    KDL::Frame trans = KDL::Frame(R, mid_frame.p); // 两个坐标系之间的完整关系
    std::cout << U.x() << ' ' << U.y() << ' ' << U.z() << ' ' << std::endl;
    std::cout << R(0, 0) << ' ' << R(0, 1) << ' ' << R(0, 2) << ' ' << std::endl;

    // 求旋转平面上三点坐标以及圆弧的圆心与半径
    KDL::Vector P1_1 = (trans.Inverse() * init_frame).p; //UVW中起点坐标,一定为(P1P2,0,0)
    KDL::Vector P2_1 = (trans.Inverse() * mid_frame).p; //UVW中中点坐标，一定为(0,0,0)
    KDL::Vector P3_1 = (trans.Inverse() * end_frame).p; //UVW中终点坐标,W一定为0
    //求圆心与半径,原点位于P1_1与P2_1中垂线上，因此U为定值，此外W为定值，设方程求解其位置
    KDL::Vector circle_origin = KDL::Vector(P1_1.x() / 2.,
                                            (pow(P3_1.x(), 2) + pow(P3_1.y(), 2) - P1_1.x() * P3_1.x()) / 2. / P3_1.y(),
                                            0.);
    double radius = pow(pow(circle_origin.x(), 2) + pow(circle_origin.y(), 2), 0.5);
    std::cout << "radius: " << radius << std::endl;
    // 求旋转平面上三点对应旋转角度,acos返回值处于0-pi
    // 限制条件：角度单调变化，中间角度位于两者之间；总变化范围不超过2pi
    // 因此对两种不单调情况进行i修正。
    double init_angle = P1_1.y() >= circle_origin.y() ? acos((P1_1.x() - circle_origin.x()) / radius) :
                        -acos((P1_1.x() - circle_origin.x()) / radius);
    double mid_angle = P2_1.y() >= circle_origin.y() ? acos((P2_1.x() - circle_origin.x()) / radius) :
                       -acos((P2_1.x() - circle_origin.x()) / radius);
    double end_angle = P3_1.y() >= circle_origin.y() ? acos((P3_1.x() - circle_origin.x()) / radius) :
                       -acos((P3_1.x() - circle_origin.x()) / radius);
    double mid_angle_possible_list[3] = {mid_angle - 2 * M_PI, mid_angle, mid_angle + 2 * M_PI};
    double end_angle_possible_list[3] = {end_angle - 2 * M_PI, end_angle, end_angle + 2 * M_PI};
    for (double i: mid_angle_possible_list)
        for (double j: end_angle_possible_list)
            if (((init_angle > i) && (i > j)) || ((init_angle < i) && (i < j)))
                if (abs(j - init_angle) < 2 * M_PI) {
                    mid_angle = i;
                    end_angle = j;
                    std::cout << "angle: " << abs(end_angle - init_angle) * 180 / M_PI << std::endl;
                }


    // 角速度，Slerp方法
    double w1, x1, y1, z1, w2, x2, y2, z2;
    init_frame.M.GetQuaternion(x1, y1, z1, w1);
    end_frame.M.GetQuaternion(x2, y2, z2, w2);
    double cos_rot_error = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2;
    if (cos_rot_error < 0.0f) { // 四元数点积的结果是负值（夹角大于90°）插值会走远路，该操作保证旋转为最短路径
        x2 = -x2;
        y2 = -y2;
        z2 = -z2;
        w2 = -w2;
        cos_rot_error = -cos_rot_error;
    }
    double sin_rot_error = sqrt(1. - cos_rot_error * cos_rot_error);
    double rot_error = atan2(sin_rot_error, cos_rot_error); // 返回以弧度为单位的角度，范围为[-pi,+pi]
    double velocity_w_para1, velocity_w_para2;
    std::cout << "rot_error(degree): " << rot_error * 180. / M_PI << std::endl;

    // 轨迹上每一个时刻，及到达的关节位置
    int dot_idx;
    double current_time;
    double percent; //当前速度积分值（0-1）
    double angle; //当前角度
    KDL::Frame current_frame_desire; //当前期望坐标系
    KDL::Frame current_frame_desire_1; //当前期望坐标系在平面坐标系中表示
    KDL::JntArray current_qpos = KDL::JntArray(joint_number); //current_frame_desire求解得到逆解current_qpos
    KDL::JntArray last_qpos = KDL::JntArray(joint_number); // 上一次的逆解角度，用于逆动力学方便求解
    std::cout << std::endl;

    // 起点
    std::cout << "######## start ########" << std::endl;
    dot_idx = 0;
    current_time = dot_idx * delta_time;
    percent = 0.0;
    std::cout << 100 * percent << " % accomplished" << std::endl;
    angle = init_angle;
    current_frame_desire_1 = trans.Inverse() * init_frame; //当前期望坐标系
    current_frame_desire = init_frame; //当前期望坐标系
    ikine_solver.CartToJnt(KDL::JntArray(joint_number), init_frame, current_qpos);
    SaveToArray(current_time, current_qpos, result_list[dot_idx]);
    last_qpos = current_qpos; // 上一次的逆解角度

    //加速
    std::cout << "######## speed up ########" << std::endl;
    for (; dot_idx * delta_time < velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4. * pow(current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        angle = init_angle + percent * (end_angle - init_angle);
        current_frame_desire_1.p = KDL::Vector(circle_origin.x() + radius * cos(angle),
                                               circle_origin.y() + radius * sin(angle), 0.0);
        current_frame_desire.p = trans * current_frame_desire_1.p;
        if (cos_rot_error > 0.9995) {
            velocity_w_para1 = 1. - percent;
            velocity_w_para2 = percent;
        } else {
            velocity_w_para1 = sin((1. - percent) * rot_error) / sin_rot_error;
            velocity_w_para2 = sin(percent * rot_error) / sin_rot_error;
        }
        current_frame_desire.M = KDL::Rotation::Quaternion(velocity_w_para1 * x1 + velocity_w_para2 * x2,
                                                           velocity_w_para1 * y1 + velocity_w_para2 * y2,
                                                           velocity_w_para1 * z1 + velocity_w_para2 * z2,
                                                           velocity_w_para1 * w1 + velocity_w_para2 * w2);
        ikine_solver.CartToJnt(last_qpos, current_frame_desire, current_qpos);
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
        last_qpos = current_qpos;
    }
    //匀速
    std::cout << "######## stable ########" << std::endl;
    for (; dot_idx * delta_time < time - velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4 * pow(velocity_raising_time, 4) + curve_max * (current_time - velocity_raising_time);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        angle = init_angle + percent * (end_angle - init_angle);
        current_frame_desire_1.p = KDL::Vector(circle_origin.x() + radius * cos(angle),
                                               circle_origin.y() + radius * sin(angle), 0.0);
        current_frame_desire.p = trans * current_frame_desire_1.p;
        if (cos_rot_error > 0.9995) {
            velocity_w_para1 = 1. - percent;
            velocity_w_para2 = percent;
        } else {
            velocity_w_para1 = sin((1. - percent) * rot_error) / sin_rot_error;
            velocity_w_para2 = sin(percent * rot_error) / sin_rot_error;
        }
        current_frame_desire.M = KDL::Rotation::Quaternion(velocity_w_para1 * x1 + velocity_w_para2 * x2,
                                                           velocity_w_para1 * y1 + velocity_w_para2 * y2,
                                                           velocity_w_para1 * z1 + velocity_w_para2 * z2,
                                                           velocity_w_para1 * w1 + velocity_w_para2 * w2);
        ikine_solver.CartToJnt(last_qpos, current_frame_desire, current_qpos);
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
        last_qpos = current_qpos;
    }
    //减速
    std::cout << "######## speed down ########" << std::endl;
    for (; dot_idx * delta_time < time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = 2 * curve_para / 4 * pow(velocity_raising_time, 4) +
                  curve_max * (time - 2 * velocity_raising_time) -
                  curve_para / 4 * pow(time - current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        angle = init_angle + percent * (end_angle - init_angle);
        current_frame_desire_1.p = KDL::Vector(circle_origin.x() + radius * cos(angle),
                                               circle_origin.y() + radius * sin(angle), 0.0);
        current_frame_desire.p = trans * current_frame_desire_1.p;
        if (cos_rot_error > 0.9995) {
            velocity_w_para1 = 1. - percent;
            velocity_w_para2 = percent;
        } else {
            velocity_w_para1 = sin((1. - percent) * rot_error) / sin_rot_error;
            velocity_w_para2 = sin(percent * rot_error) / sin_rot_error;
        }
        current_frame_desire.M = KDL::Rotation::Quaternion(velocity_w_para1 * x1 + velocity_w_para2 * x2,
                                                           velocity_w_para1 * y1 + velocity_w_para2 * y2,
                                                           velocity_w_para1 * z1 + velocity_w_para2 * z2,
                                                           velocity_w_para1 * w1 + velocity_w_para2 * w2);
        ikine_solver.CartToJnt(last_qpos, current_frame_desire, current_qpos);
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
        last_qpos = current_qpos;
    }
    std::cout << "end" << std::endl;

    if (save_file) {
        SaveToFile(result_list, init_frame, end_frame, time, dot_num, velocity_raising_percentage, velocity_limit,
                   "Circle", file_name);
    }

    return result_list;
}

/*!
 * 关节空间路径规划，即只在乎在关节空间上丝滑的到达终点，不考虑中间点
 * @param init_frame 初始位姿
 * @param end_frame 结束位姿
 * @param time 运动总时间，秒
 * @param dot_num 插值点个数
 * @param velocity_raising_percentage 加速时间占运行时间比例
 * @param velocity_limit 线速度的最大速度限制，如果加速时间占比不合理将导致最大速度超过速度限制，米每秒
 * @param save_file 保存文件标志位
 * @param file_name 保存文件路径名
 * @return 插值结果数组首地址
 */
double **Robot::PathToPoint(const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time,
                            int dot_num, double velocity_raising_percentage, double velocity_limit, bool save_file,
                            std::string file_name) {
    // 运动学求解器
    KDL::ChainIkSolverPos_LMA ikine_solver = KDL::ChainIkSolverPos_LMA(kdl_model, 1E-5, 20000);

    // 利用运动的距离、时间，求出速度的参数以及相关参数
    double **result_list = CreateResultList(dot_num);
    double delta_time = time / dot_num;
    double velocity_raising_time = velocity_raising_percentage * time; //加速所用时间
    double curve_para = 1. / (-3. / 2. * pow(velocity_raising_time, 4) + time * pow(velocity_raising_time, 3)); //样条曲线参数
    double curve_max = curve_para * pow(velocity_raising_time, 3); //速度梯形的高
    std::cout << "curve: para " << curve_para << " max: " << curve_max << std::endl;

    KDL::JntArray init_qpos = KDL::JntArray(joint_number);
    ikine_solver.CartToJnt(KDL::JntArray(joint_number), init_frame, init_qpos);
    KDL::JntArray end_qpos = KDL::JntArray(joint_number);
    ikine_solver.CartToJnt(KDL::JntArray(joint_number), end_frame, end_qpos);

    // 轨迹上每一个时刻，及到达的关节位置
    int dot_idx;
    double current_time;
    double percent; //当前速度积分值（0-1）
    KDL::JntArray current_qpos = KDL::JntArray(joint_number);
    std::cout << std::endl;

    // 起点
    std::cout << "######## start ########" << std::endl;
    dot_idx = 0;
    current_time = dot_idx * delta_time;
    percent = 0.0;
    std::cout << 100 * percent << " % accomplished" << std::endl;
    for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
        current_qpos(joint_idx) = init_qpos(joint_idx);
    }
    SaveToArray(current_time, current_qpos, result_list[dot_idx]);

    //加速
    std::cout << "######## speed up ########" << std::endl;
    for (; dot_idx * delta_time < velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4. * pow(current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            current_qpos(joint_idx) = init_qpos(joint_idx) + percent * (end_qpos(joint_idx) - init_qpos(joint_idx));
        }
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
    }
    //匀速
    std::cout << "######## stable ########" << std::endl;
    for (; dot_idx * delta_time < time - velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4 * pow(velocity_raising_time, 4) + curve_max * (current_time - velocity_raising_time);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            current_qpos(joint_idx) = init_qpos(joint_idx) + percent * (end_qpos(joint_idx) - init_qpos(joint_idx));
        }
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
    }
    //减速
    std::cout << "######## speed down ########" << std::endl;
    for (; dot_idx * delta_time < time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = 2 * curve_para / 4 * pow(velocity_raising_time, 4) +
                  curve_max * (time - 2 * velocity_raising_time) -
                  curve_para / 4 * pow(time - current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            current_qpos(joint_idx) = init_qpos(joint_idx) + percent * (end_qpos(joint_idx) - init_qpos(joint_idx));
        }
        SaveToArray(current_time, current_qpos, result_list[dot_idx]);
    }
    std::cout << "end" << std::endl;

    if (save_file) {
        SaveToFile(result_list, init_frame, end_frame, time, dot_num, velocity_raising_percentage, velocity_limit,
                   "Line", file_name);
    }

    return result_list;
}

void Robot::ShowXPos(KDL::Frame frame) {
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    std::cout << "pos: " << frame.p[0] << " " << frame.p[1] << " " << frame.p[2] << std::endl;
    std::cout << "quat: " << x << " " << y << " " << z << " " << w << std::endl;
}

void Robot::ShowQPos(KDL::JntArray qpos) {
    for (int joint_idx = 0; joint_idx < this->joint_number; joint_idx++) {
        std::cout << qpos.data[joint_idx] << " ";
    }
    std::cout << std::endl;
}

void Robot::SaveToArray(double current_time, KDL::JntArray current_qpos, double *result_array) {
    KDL::ChainFkSolverPos_recursive fkine_solver = KDL::ChainFkSolverPos_recursive(kdl_model);
    KDL::Frame current_frame_real; // current_qpos前向运动学得到实际角度
    fkine_solver.JntToCart(current_qpos, current_frame_real);
    double x, y, z, w;
    current_frame_real.M.GetQuaternion(x, y, z, w);
//    std::cout << velocity_w_para1 << ' ' << velocity_w_para2 << std::endl;
//    ShowQPos(current_qpos);
//    ShowXPos(current_frame_real);
//    std::cout << std::endl;
    result_array[0] = current_time;
    for (int joint_idx = 0; joint_idx < joint_number + 1; joint_idx++) {
        result_array[joint_idx + 1] = current_qpos(joint_idx);
    }
    for (int pos_idx = 0; pos_idx < 3; pos_idx++) {
        result_array[pos_idx + 1 + joint_number] = current_frame_real.p(pos_idx);
    }
    result_array[1 + joint_number + 3 + 0] = x;
    result_array[1 + joint_number + 3 + 1] = y;
    result_array[1 + joint_number + 3 + 2] = z;
    result_array[1 + joint_number + 3 + 3] = w;
}

void Robot::SaveToFile(double **result_list, const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time,
                       int dot_num, double velocity_raising_percentage, double speed_limit, std::string method,
                       std::string file_name) {
    std::string result_dir = "../result/";
    std::string file_dir = "../result/" + file_name;
    if (access(file_dir.c_str(), F_OK) == -1) {
        mkdir(result_dir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        mkdir(file_dir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    }

    // 保存关节角度信息
    std::ofstream qpos_file;
    qpos_file.open(file_dir + "/qpos.csv");
    for (int i = 0; i < dot_num; i++) {
        for (int j = 0; j < 1 + this->joint_number + 3 + 4; j++) {
            qpos_file << result_list[i][j] << ',';
        }
        qpos_file << std::endl;
    }
    qpos_file.close();

    // 保存配置信息
    double x, y, z, w;
    std::ofstream profile;
    profile.open(file_dir + "/profile.txt");
    profile << "method:\t" << method << std::endl;

    profile << "init frame:\t" << std::endl;
    profile << "pos:\t" << init_frame.p.x() << ' ' << init_frame.p.y() << ' ' << init_frame.p.z() << ' ' << std::endl;
    init_frame.M.GetQuaternion(x, y, z, w);
    profile << "quat:\t" << x << " " << y << " " << z << " " << w << std::endl;

    profile << "end frame:" << std::endl;
    profile << "pos:\t" << end_frame.p.x() << ' ' << end_frame.p.y() << ' ' << end_frame.p.z() << ' ' << std::endl;
    end_frame.M.GetQuaternion(x, y, z, w);
    profile << "quat:\t" << x << " " << y << " " << z << " " << w << std::endl;

    profile << "time:\t" << time << std::endl;
    profile << "dot num:\t" << dot_num << std::endl;
    profile << "vel raise percentage:\t" << velocity_raising_percentage << std::endl;
    profile << "speed limit:\t" << speed_limit << std::endl;

    profile.close();
}
