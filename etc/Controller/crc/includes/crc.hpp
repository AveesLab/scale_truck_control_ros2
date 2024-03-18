#include <fstream>
#include "zmq_class.h"

namespace CentralResiliencyCoordinator{

class CentralRC{
  public:
    CentralRC();
    ~CentralRC();

    struct timeval launch_time_;
    void communicate();
    bool is_node_running_;

  private:
    ZMQ_CLASS ZMQ_SOCKET_;

    void init();
    void reply(ZmqData* zmq_data);
    void estimateVelocity(uint8_t index);
    void statusCheck(ZmqData *lv_data, ZmqData *fv1_data, ZmqData *fv2_data);
    void recordData(struct timeval *time);
    void printStatus();
    void updateData(ZmqData* zmq_data);
    bool getSamplingTime(float cur_dist, float prev_dist, int idx);
    float lowPassFilter(float sampling_time, float pred_vel);

    bool time_flag_;
    uint8_t index_;
    uint8_t crc_mode_;

    ZmqData *lv_data_, *fv1_data_, *fv2_data_;

    float tau_ = 0.5f;
    float fv1_est_vel_tmp_ = 0.f;
    float prev_res_ = 0.8f;

    float fv1_prev_dist_, fv2_prev_dist_;
    float lv_est_vel_, fv1_est_vel_, fv2_est_vel_;
    float sampling_time1_, sampling_time2_;
    std::string log_path_ = "/home/avees/logfiles/";

    struct timeval start_time1_, start_time2_, end_time1_, end_time2_;
    double time_;

    std::thread repThread0_, repThread1_, repThread2_;
    std::mutex data_mutex_;
};

}

