//
// Created by yzk on 11/8/22.
//

#include "../include/jk5_stick.h"
#include "iostream"

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

double **Robot::CreateResultList(int dot_num) {
    auto **result_list = new double *[dot_num];
    for (int dot_idx = 0; dot_idx < dot_num; dot_idx++) {
        result_list[dot_idx] = new double[this->joint_number + 1]{0.0};//时间+各关节
    }

    return result_list;
}

/*!
 * 迪卡尔空间下直线路径规划
 * 中间位置通过直线插值确定，中间姿态通过Slerp插值确定
 * 具体位姿由梯形的速度曲线确定，两条斜边为完全相同的三次样条曲线，保证起始与结束的速度加速度为0
 * 速度曲线积分得到的面积为1，当前时刻位姿由0到该时刻积分面积确定，每个方向上的角速度与线速度同步变化，即同时达到最大，同时开始减速
 * @param result_list 行为为各时刻，共dot_num行；列为到达该点的时间与各关节的角度，共joint_number+1列
 * @param init_frame 初始位姿
 * @param end_frame 结束位姿
 * @param time 运动总时间，秒
 * @param dot_num 插值点个数
 * @param speed_up_percentage 加速时间占运行时间比例
 * @param speed_limit 线速度的最大速度限制，如果加速时间占比不合理将导致最大速度超过速度限制，米每秒
 */
void Robot::PathAlongLine(double **result_list, const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time,
                          int dot_num, double velocity_raising_percentage, double velocity_limit) {
    // 运动学求解器
    KDL::ChainFkSolverPos_recursive fkine_solver = KDL::ChainFkSolverPos_recursive(kdl_model);
    KDL::ChainIkSolverPos_LMA ikine_solver = KDL::ChainIkSolverPos_LMA(kdl_model, 1E-5, 20000);

    // 利用运动的距离、时间，求出速度的参数以及相关参数
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
    KDL::Frame current_frame_real; // current_qpos前向运动学得到实际角度
    KDL::JntArray last_qpos = KDL::JntArray(joint_number); // 上一次的逆解角度，用于逆动力学方便求解

    // 起点
    std::cout << "######## start ########" << std::endl;
    dot_idx = 0;
    current_time = dot_idx * delta_time;
    percent = 0.0;
    std::cout << 100 * percent << " % accomplished" << std::endl;
    current_frame_desire = init_frame; //当前期望坐标系
    ikine_solver.CartToJnt(KDL::JntArray(joint_number), init_frame, current_qpos);
    fkine_solver.JntToCart(current_qpos, current_frame_real);
//    ShowQPos(current_qpos);
    ShowXPos(current_frame_real);
    std::cout << std::endl;
    result_list[0][0] = current_time;
    for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
        result_list[0][joint_idx + 1] = current_qpos(joint_idx);
    }
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
        fkine_solver.JntToCart(current_qpos, current_frame_real);
//        std::cout << velocity_w_para1 << ' ' << velocity_w_para2 << std::endl;
        //ShowQPos(current_qpos);
        ShowXPos(current_frame_real);
        std::cout << std::endl;
        result_list[dot_idx][0] = current_time;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            result_list[dot_idx][joint_idx + 1] = current_qpos(joint_idx);
        }
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
        fkine_solver.JntToCart(current_qpos, current_frame_real);
//        std::cout << velocity_w_para1 << ' ' << velocity_w_para2 << std::endl;
        //ShowQPos(current_qpos);
        ShowXPos(current_frame_real);
        std::cout << std::endl;
        result_list[dot_idx][0] = current_time;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            result_list[dot_idx][joint_idx + 1] = current_qpos(joint_idx);
        }
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
        fkine_solver.JntToCart(current_qpos, current_frame_real);
//        std::cout << velocity_w_para1 << ' ' << velocity_w_para2 << std::endl;
        //ShowQPos(current_qpos);
        ShowXPos(current_frame_real);
        std::cout << std::endl;
        result_list[dot_idx][0] = current_time;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            result_list[dot_idx][joint_idx + 1] = current_qpos(joint_idx);
        }
        last_qpos = current_qpos;
    }
    std::cout << "end" << std::endl;
}

void Robot::PathAlongCircle() {

}

void Robot::PathToPoint(double **result_list, const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time, int dot_num,
           double velocity_raising_percentage, double speed_limit) {
    // 运动学求解器
    KDL::ChainFkSolverPos_recursive fkine_solver = KDL::ChainFkSolverPos_recursive(kdl_model);
    KDL::ChainIkSolverPos_LMA ikine_solver = KDL::ChainIkSolverPos_LMA(kdl_model, 1E-5, 20000);

    // 利用运动的距离、时间，求出速度的参数以及相关参数
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
    KDL::Frame current_frame_real; // current_qpos前向运动学得到实际角度
    std::cout << std::endl;

    // 起点
    std::cout << "######## start ########" << std::endl;
    dot_idx = 0;
    current_time = dot_idx * delta_time;
    percent = 0.0;
    std::cout << 100 * percent << " % accomplished" << std::endl;
    result_list[0][0] = current_time;
    for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
        current_qpos(joint_idx) = init_qpos(joint_idx);
        result_list[0][joint_idx + 1] = current_qpos(joint_idx);
    }
    fkine_solver.JntToCart(current_qpos, current_frame_real);
    ShowXPos(current_frame_real);
//    ShowQPos(current_qpos);
    std::cout << std::endl;

    //加速
    std::cout << "######## speed up ########" << std::endl;
    for (; dot_idx * delta_time < velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4. * pow(current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        result_list[dot_idx][0] = current_time;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            current_qpos(joint_idx) = init_qpos(joint_idx) + percent * (end_qpos(joint_idx) - init_qpos(joint_idx));
            result_list[dot_idx][joint_idx + 1] = current_qpos(joint_idx);
        }
        fkine_solver.JntToCart(current_qpos, current_frame_real);
        ShowXPos(current_frame_real);
//        ShowQPos(current_qpos);
        std::cout << std::endl;
    }
    //匀速
    std::cout << "######## stable ########" << std::endl;
    for (; dot_idx * delta_time < time - velocity_raising_time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = curve_para / 4 * pow(velocity_raising_time, 4) + curve_max * (current_time - velocity_raising_time);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        result_list[dot_idx][0] = current_time;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            current_qpos(joint_idx) = init_qpos(joint_idx) + percent * (end_qpos(joint_idx) - init_qpos(joint_idx));
            result_list[dot_idx][joint_idx + 1] = current_qpos(joint_idx);
        }
        fkine_solver.JntToCart(current_qpos, current_frame_real);
        ShowXPos(current_frame_real);
//        ShowQPos(current_qpos);
        std::cout << std::endl;
    }
    //减速
    std::cout << "######## speed down ########" << std::endl;
    for (; dot_idx * delta_time < time; dot_idx++) {
        current_time = dot_idx * delta_time;
        percent = 2 * curve_para / 4 * pow(velocity_raising_time, 4) +
                  curve_max * (time - 2 * velocity_raising_time) -
                  curve_para / 4 * pow(time - current_time, 4);
        std::cout << 100 * percent << " % accomplished" << std::endl;
        result_list[dot_idx][0] = current_time;
        for (int joint_idx = 0; joint_idx < joint_number; joint_idx++) {
            current_qpos(joint_idx) = init_qpos(joint_idx) + percent * (end_qpos(joint_idx) - init_qpos(joint_idx));
            result_list[dot_idx][joint_idx + 1] = current_qpos(joint_idx);
        }
        fkine_solver.JntToCart(current_qpos, current_frame_real);
        ShowXPos(current_frame_real);
//        ShowQPos(current_qpos);
        std::cout << std::endl;
    }
    std::cout << "end" << std::endl;
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


