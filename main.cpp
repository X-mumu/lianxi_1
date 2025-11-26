#include <iostream>
#include <vector>
#include "dynamic_obstacle_tracker.h"

using namespace std;

int main() {
    DynamicObstacleTracker tracker(/*max_lost_frames*/ 5);

    // 假设当前时间为 0.0 秒，有两个障碍物观测
    vector<ObstacleObservation> obs1{
        {1, {10.0, 5.0}, 0.0},
        {2, {20.0, -3.0}, 0.0}
    };
    tracker.update(obs1);

    // 过 0.1 秒，又有一帧观测
    vector<ObstacleObservation> obs2{
        {1, {10.5, 5.0}, 0.1},  // 1 号障碍物向 x 正方向移动
        // 2 号本帧未观测到，相当于丢失一帧
    };
    tracker.update(obs2);

    // 预测到 0.5 秒时刻的障碍物状态
    tracker.predict_to(0.5);

    auto states = tracker.get_all_states();
    for (const auto& st : states) {
        cout << "Obstacle " << st.id
             << " pos=(" << st.position.x << ", " << st.position.y << ")"
             << " vel=(" << st.velocity.x << ", " << st.velocity.y << ")"
             << " lost_frames=" << st.lost_frame_count
             << endl;
    }

    return 0;
}