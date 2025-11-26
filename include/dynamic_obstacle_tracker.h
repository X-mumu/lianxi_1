#pragma once
#include <unordered_map>
#include <vector>
#include <chrono>

struct Vec2 {
    double x{0.0};
    double y{0.0};
};

struct ObstacleState {
    int id{-1};                    // 障碍物 ID
    Vec2 position;                 // 位置 (m)
    Vec2 velocity;                 // 速度 (m/s)
    double last_update_time{0.0};  // 最近一次更新时间 (秒，参考统一时钟)
    int lost_frame_count{0};       // 连续丢失帧数
};

struct ObstacleObservation {
    int id;            // 观测 ID（来自感知模块：跟踪 ID / track id）
    Vec2 position;     // 当前观测位置
    double timestamp;  // 当前时间（秒）
};

class DynamicObstacleTracker {
public:
    // max_lost_frames: 超过多少帧未观测到就删除该障碍物
    explicit DynamicObstacleTracker(int max_lost_frames = 10,
                                    double alpha_pos = 0.7,
                                    double alpha_vel = 0.5);

    // 用一帧的观测结果更新所有障碍物状态
    void update(const std::vector<ObstacleObservation>& observations);

    // 基于当前时间戳，对所有障碍物做前向预测（常速度模型）
    void predict_to(double timestamp);

    // 获取当前所有障碍物状态
    std::vector<ObstacleState> get_all_states() const;

    // 根据 ID 获取某个障碍物状态；不存在返回 false
    bool get_state(int id, ObstacleState& out) const;

private:
    // 内部使用：根据 ID 找/创 建障碍物
    ObstacleState& get_or_create(int id, double timestamp, const Vec2& pos);

    // 清理长期未更新的障碍物
    void cleanup();

private:
    std::unordered_map<int, ObstacleState> obstacles_;
    int max_lost_frames_;   // 最大允许连续丢失帧数
    double alpha_pos_;      // 位置更新的 EMA 系数（0~1）
    double alpha_vel_;      // 速度更新的 EMA 系数（0~1）
};