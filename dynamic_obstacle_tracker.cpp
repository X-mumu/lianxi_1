#include "dynamic_obstacle_tracker.h"
#include <cmath>

DynamicObstacleTracker::DynamicObstacleTracker(int max_lost_frames,
                                               double alpha_pos,
                                               double alpha_vel)
    : max_lost_frames_(max_lost_frames),
      alpha_pos_(alpha_pos),
      alpha_vel_(alpha_vel) {}

// 简单常速度预测
static void predict_state(ObstacleState& st, double new_time) {
    double dt = new_time - st.last_update_time;
    if (dt <= 0.0) return;
    st.position.x += st.velocity.x * dt;
    st.position.y += st.velocity.y * dt;
    st.last_update_time = new_time;
}

ObstacleState& DynamicObstacleTracker::get_or_create(
    int id, double timestamp, const Vec2& pos) {
    auto it = obstacles_.find(id);
    if (it == obstacles_.end()) {
        ObstacleState st;
        st.id = id;
        st.position = pos;
        st.velocity = {0.0, 0.0};
        st.last_update_time = timestamp;
        st.lost_frame_count = 0;
        auto res = obstacles_.emplace(id, st);
        return res.first->second;
    }
    return it->second;
}

void DynamicObstacleTracker::update(
    const std::vector<ObstacleObservation>& observations) {

    // 标记：先默认所有障碍物都丢失一帧，后面如果被观测到再修正
    for (auto& kv : obstacles_) {
        kv.second.lost_frame_count++;
    }

    // 处理当前帧的观测
    for (const auto& obs : observations) {
        auto& st = get_or_create(obs.id, obs.timestamp, obs.position);

        // 先根据时间做一次预测
        predict_state(st, obs.timestamp);

        // 更新速度估计：v ≈ (p_obs - p_old) / dt，然后用 EMA
        double dt = obs.timestamp - st.last_update_time;
        // 注意：predict_state 已经把 last_update_time 更新为 obs.timestamp，
        // 这里重新算 dt 就用一个小 epsilon 保护
        if (dt <= 0.0) dt = 1e-3;

        Vec2 meas_vel{
            (obs.position.x - st.position.x) / dt,
            (obs.position.y - st.position.y) / dt
        };

        // 位置：简单 EMA 融合
        st.position.x = alpha_pos_ * obs.position.x +
                        (1.0 - alpha_pos_) * st.position.x;
        st.position.y = alpha_pos_ * obs.position.y +
                        (1.0 - alpha_pos_) * st.position.y;

        // 速度：EMA 融合
        st.velocity.x = alpha_vel_ * meas_vel.x +
                        (1.0 - alpha_vel_) * st.velocity.x;
        st.velocity.y = alpha_vel_ * meas_vel.y +
                        (1.0 - alpha_vel_) * st.velocity.y;

        st.last_update_time = obs.timestamp;
        st.lost_frame_count = 0;  // 本帧有观测，清零
    }

    // 删除长期未观测到的障碍物
    cleanup();
}

void DynamicObstacleTracker::predict_to(double timestamp) {
    for (auto& kv : obstacles_) {
        predict_state(kv.second, timestamp);
    }
}

std::vector<ObstacleState> DynamicObstacleTracker::get_all_states() const {
    std::vector<ObstacleState> result;
    result.reserve(obstacles_.size());
    for (const auto& kv : obstacles_) {
        result.push_back(kv.second);
    }
    return result;
}

bool DynamicObstacleTracker::get_state(int id, ObstacleState& out) const {
    auto it = obstacles_.find(id);
    if (it == obstacles_.end()) return false;
    out = it->second;
    return true;
}

void DynamicObstacleTracker::cleanup() {
    std::vector<int> to_remove;
    to_remove.reserve(obstacles_.size());
    for (const auto& kv : obstacles_) {
        if (kv.second.lost_frame_count > max_lost_frames_) {
            to_remove.push_back(kv.first);
        }
    }
    for (int id : to_remove) {
        obstacles_.erase(id);
    }
}