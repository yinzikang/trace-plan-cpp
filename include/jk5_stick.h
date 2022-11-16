//
// Created by yzk on 11/8/22.
//

#ifndef TRACE_PLAN_CPP_JK5_H
#define TRACE_PLAN_CPP_JK5_H

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"

class Robot {
public:
    KDL::Chain kdl_model;
    unsigned int joint_number;

    Robot();

    double **CreateResultList(int dot_num);

    void ShowXPos(KDL::Frame frame);

    void ShowQPos(KDL::JntArray qpos);

    // 迪卡尔空间下直线路径
    double **PathAlongLine(const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time,
                           int dot_num,
                           double velocity_raising_percentage, double velocity_limit, bool save_fig = true,
                           std::string file_name = "test");

    // 迪卡尔空间下圆弧路径
    void PathAlongCircle();

    // 关节空间下运动到指定位姿
    double **PathToPoint(const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time,
                         int dot_num,
                         double velocity_raising_percentage, double velocity_limit, bool save_fig = true,
                         std::string file_name = "test");

private:
    void SaveToFile(double **result_list, const KDL::Frame &init_frame, const KDL::Frame &end_frame, double time,
                    int dot_num,
                    double velocity_raising_percentage, double velocity_limit, std::string method,
                    std::string file_name);

    void SaveToArray(double current_time, KDL::JntArray qpos, double *result_array);
};


#endif //TRACE_PLAN_CPP_JK5_H
