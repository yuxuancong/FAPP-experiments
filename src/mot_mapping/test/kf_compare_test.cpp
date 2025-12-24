#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "kf_compare.hpp"

// === 运动模式定义 ===
enum MotionType {
    CONSTANT_VELOCITY = 0,  // 匀速直线运动
    SINUSOIDAL = 1,         // 正弦变速运动（与仿真env1一致）
    VARIABLE_ACCEL = 2      // 变加速运动
};

// 生成模拟轨迹
// motion_type: 0=匀速, 1=正弦变速, 2=变加速
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> generateTrajectory(double dt, int N, int motion_type) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> traj;

    // 初始位置
    Eigen::Vector3d pos(0.0, 0.0, 1.0);
    Eigen::Vector3d vel(0.0, 0.0, 0.0);

    // 参数（参考env1: vel_amp 0.2~2.0, vel_per 0~2.0）
    double vel_amp_x = 1.5, vel_amp_y = 1.2;
    double vel_per_x = 1.0, vel_per_y = 0.8;
    double const_vx = 1.0, const_vy = 0.5;
    double accel = 0.5;

    for (int i = 0; i < N; ++i) {
        double t = i * dt;
        double vx, vy;

        switch (motion_type) {
            case CONSTANT_VELOCITY:
                // 匀速直线运动
                vx = const_vx;
                vy = const_vy;
                break;
            case SINUSOIDAL:
                // 正弦变速运动（与random_forest_sensing.cpp ObjUpdate一致）
                vx = vel_amp_x * sin(vel_per_x * t) + 1.1 * vel_amp_x;
                vy = vel_amp_y * sin(vel_per_y * t) + 1.1 * vel_amp_y;
                break;
            case VARIABLE_ACCEL:
                // 变加速运动（加速度随时间变化）
                vx = accel * t * cos(0.3 * t);
                vy = accel * t * sin(0.3 * t);
                break;
            default:
                vx = vy = 0;
        }

        vel = Eigen::Vector3d(vx, vy, 0.0);
        if (i > 0) {
            pos += vel * dt;
        }
        traj.push_back({pos, vel});
    }
    return traj;
}

// 添加高斯噪声（支持时变噪声）
Eigen::Vector3d addNoise(const Eigen::Vector3d& v, double sigma) {
    Eigen::Vector3d n;
    for (int i = 0; i < 3; ++i)
        n(i) = v(i) + sigma * ((double)rand() / RAND_MAX - 0.5) * 2;
    return n;
}

const char* motionName(int type) {
    switch(type) {
        case 0: return "匀速直线运动";
        case 1: return "正弦变速运动";
        case 2: return "变加速运动";
        case 3: return "正弦变速+噪声突变";
        default: return "未知";
    }
}

const char* motionNameEn(int type) {
    switch(type) {
        case 0: return "constant_velocity";
        case 1: return "sinusoidal";
        case 2: return "variable_accel";
        case 3: return "sinusoidal_noise_burst";
        default: return "unknown";
    }
}

int main() {
    double dt = 0.02;
    int N = 500;
    double pos_noise_base = 0.3;
    double vel_noise_base = 0.5;

    std::cout << "====== 四种运动状态KF对比测试 ======\n\n";

    for (int motion_type = 0; motion_type <= 3; ++motion_type) {
        // 对于噪声突变场景，使用正弦运动
        int traj_type = (motion_type == 3) ? 1 : motion_type;
        auto traj = generateTrajectory(dt, N, traj_type);

        StandardKF skf(dt);
        AdaptiveKF akf(dt, 10);

        skf.reset(traj[0].first);
        akf.reset(traj[0].first);

        double rmse_skf_pos = 0, rmse_akf_pos = 0;
        double rmse_skf_vel = 0, rmse_akf_vel = 0;

        // 输出CSV文件用于绘图
        std::string filename = std::string("traj_") + motionNameEn(motion_type) + ".csv";
        std::ofstream ofs(filename);
        ofs << "step,t,gt_x,gt_y,skf_x,skf_y,akf_x,akf_y,skf_err,akf_err,noise_level\n";

        for (int i = 1; i < N; ++i) {
            skf.predict();
            akf.predict();

            // 噪声突变：在200-350步之间噪声增大5倍（模拟遮挡/干扰）
            double pos_noise = pos_noise_base;
            double vel_noise = vel_noise_base;
            if (motion_type == 3 && i >= 200 && i < 350) {
                pos_noise = pos_noise_base * 5.0;
                vel_noise = vel_noise_base * 5.0;
            }

            Eigen::Vector3d z_pos = addNoise(traj[i].first, pos_noise);
            Eigen::Vector3d z_vel = addNoise(traj[i].second, vel_noise);

            skf.update(z_pos, z_vel);
            akf.update(z_pos, z_vel);

            double err_skf = (skf.pos() - traj[i].first).norm();
            double err_akf = (akf.pos() - traj[i].first).norm();
            rmse_skf_pos += err_skf * err_skf;
            rmse_akf_pos += err_akf * err_akf;
            rmse_skf_vel += (skf.vel() - traj[i].second).squaredNorm();
            rmse_akf_vel += (akf.vel() - traj[i].second).squaredNorm();

            ofs << i << "," << i*dt << ","
                << traj[i].first(0) << "," << traj[i].first(1) << ","
                << skf.pos()(0) << "," << skf.pos()(1) << ","
                << akf.pos()(0) << "," << akf.pos()(1) << ","
                << err_skf << "," << err_akf << "," << pos_noise << "\n";
        }
        ofs.close();

        rmse_skf_pos = std::sqrt(rmse_skf_pos / (N - 1));
        rmse_akf_pos = std::sqrt(rmse_akf_pos / (N - 1));
        rmse_skf_vel = std::sqrt(rmse_skf_vel / (N - 1));
        rmse_akf_vel = std::sqrt(rmse_akf_vel / (N - 1));

        std::cout << "【" << motionName(motion_type) << "】 -> " << filename << "\n";
        std::cout << "  标准KF   位置RMSE: " << rmse_skf_pos << "  速度RMSE: " << rmse_skf_vel << "\n";
        std::cout << "  自适应KF 位置RMSE: " << rmse_akf_pos << "  速度RMSE: " << rmse_akf_vel << "\n";
        std::cout << "  位置RMSE差异: " << (rmse_akf_pos - rmse_skf_pos) << " (" 
                  << (rmse_akf_pos < rmse_skf_pos ? "自适应更优" : "标准更优") << ")\n\n";
    }

    std::cout << "轨迹数据已保存，请运行 plot_traj.py 绘图\n";
    return 0;
}
