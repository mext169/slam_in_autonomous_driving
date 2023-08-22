//
// Created by xiang on 22-12-29.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_bool(use_quaternion, false, "是否使用四元数计算");
DEFINE_double(angular_velocity_z, 30.0, "车绕z轴固定的角速度 度/s");
DEFINE_double(initial_velocity_x, 10.0, "初始x方向速度 m/s");
DEFINE_string(save_path, "../data/ch2/motion_traj.txt", "轨迹保存路径");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false) {
        return -1;
    }

    std::ofstream out_file;
    out_file.open(FLAGS_save_path, std::ofstream::app);

    const double gravity_acc = -9.8; // 重力加速度 m/s^2
    double angular_velocity_z_rad = FLAGS_angular_velocity_z * sad::math::kDEG2RAD;  // z轴角速度转化成弧度制
    SE3 pose;                                                                        // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_z_rad);                                       // 沿z轴角速度矢量
    Vec3d v_body(FLAGS_initial_velocity_x, 0, 0);                                    // 初始x方向线速度
    Vec3d acc_world(0, 0, gravity_acc);                                              // 加速度
    const double dt = 0.05;                                                          // 每次更新的时间

    while (ui.ShouldQuit() == false) {

        // 位置受初始线速度和z方向的重力加速度影响
        // 初始线速度在车体坐标系，需要先转换到世界坐标系再使用
        // 重力加速度在世界坐标系下，可以直接使用
        Vec3d v_world = pose.so3() * v_body;
        pose.translation() += v_world * dt + 0.5 * acc_world * dt * dt;

        // 更新自身速度，需要把世界坐标系下的加速度转到车体坐标系
        Vec3d acc_body = pose.so3().inverse() * acc_world;
        v_body += acc_body * dt;

        // 更新自身旋转，旋转只受到自身角速度的影响
        if (FLAGS_use_quaternion) {
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }

        LOG(INFO) << "v-body: " << v_body.transpose();
        LOG(INFO) << "acc-body: " << acc_body.transpose();
        LOG(INFO) << "pose: " << pose.translation().transpose();
        out_file << std::setprecision(9) << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << "\n";
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);
    }

    out_file.close();
    ui.Quit();
    return 0;
}