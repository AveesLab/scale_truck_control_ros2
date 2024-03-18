#include "includes/crc.hpp"

namespace CentralResiliencyCoordinator{
  
CentralRC::CentralRC()
  : ZMQ_SOCKET_(){

  init();  
}

CentralRC::~CentralRC(){
  delete lv_data_;
  delete fv1_data_;
  delete fv2_data_;
}

void CentralRC::init(){
  is_node_running_ = true;
  index_ = 30;
  crc_mode_ = 0;

  lv_data_ = new ZmqData;
  fv1_data_ = new ZmqData;
  fv2_data_ = new ZmqData;

  lv_data_->src_index = index_;
  lv_data_->tar_index = 10;

  fv1_data_->src_index = index_;
  fv1_data_->tar_index = 11;

  fv2_data_->src_index = index_;
  fv2_data_->tar_index = 12;

  fv1_prev_dist_ = 0.8f;
  fv2_prev_dist_ = 0.8f;

  time_flag_ = false;
  sampling_time1_ = 0.1;
  sampling_time2_ = 0.1;

  if(ZMQ_SOCKET_.rep_flag0_){
    repThread0_ = std::thread(&CentralRC::reply, this, lv_data_);
  }
  if(ZMQ_SOCKET_.rep_flag1_){
    repThread1_ = std::thread(&CentralRC::reply, this, fv1_data_);
  }
  if(ZMQ_SOCKET_.rep_flag2_){
    repThread2_ = std::thread(&CentralRC::reply, this, fv2_data_);
  }
}

void CentralRC::reply(ZmqData* zmq_data){
  ZmqData tmp;
  while(is_node_running_){
    {
      std::scoped_lock lock(data_mutex_);
      tmp = *zmq_data;
      tmp.crc_mode = crc_mode_;
    }
    if(zmq_data->tar_index == 10){  //LV
      ZMQ_SOCKET_.replyZMQ(&tmp);
    }
    else if(zmq_data->tar_index == 11){  //FV1
      ZMQ_SOCKET_.replyZMQ(&tmp);
    }
    else if(zmq_data->tar_index == 12){  //FV2
      ZMQ_SOCKET_.replyZMQ(&tmp);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void CentralRC::estimateVelocity(uint8_t index){
  assert(index < 13 || index > 9);
  std::scoped_lock lock(data_mutex_);

  if (index == 10){  //LV
    if (!lv_data_->alpha){
      lv_data_->est_vel = lv_data_->cur_vel;
    }
    else if (lv_data_->alpha && !lv_data_->alpha){
        lv_data_->est_vel = ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time1_) + fv1_data_->cur_vel;
    }
    else if ((lv_data_->alpha || fv1_data_->alpha) && !fv2_data_->alpha){
        lv_data_->est_vel = ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time1_) + ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time2_) + fv2_data_->cur_vel;
    }
    else{  //All trucks' velocity sensors are fail 
      crc_mode_ = 2;
      lv_data_->crc_mode = crc_mode_;
    }
  }
  else if (index == 11){  //FV1
    float origin_est_vel;
    if (!fv1_data_->alpha){
      fv1_data_->est_vel = fv1_data_->cur_vel;
    }
    else if (fv1_data_->alpha && !lv_data_->alpha){
      origin_est_vel = ((-1.0f) * ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time1_)) + lv_data_->cur_vel;
      fv1_est_vel_tmp_ = origin_est_vel;
      fv1_data_->est_vel = lowPassFilter(sampling_time1_, origin_est_vel);
    }
    else if ((fv1_data_->alpha || lv_data_->alpha) && !fv2_data_->alpha){
      fv1_data_->est_vel = ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time2_) + fv2_data_->cur_vel;
    }
    else{  //All trucks' velocity sensors are fail
      crc_mode_ = 2;
      fv1_data_->crc_mode = crc_mode_;
    }
  }
  else if (index == 12){  //FV2
    if (!fv2_data_->alpha){
      fv2_data_->est_vel = fv2_data_->cur_vel;
    }
    else if (fv2_data_->alpha && !fv1_data_->alpha){
        fv2_data_->est_vel = ((-1.0f) * ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time2_)) + fv1_data_->cur_vel;
    }
    else if ((fv2_data_->alpha || fv1_data_->alpha) && !lv_data_->alpha){
        fv2_data_->est_vel = ((-1.0f) * ((fv1_data_->cur_dist - fv1_prev_dist_) / sampling_time1_)) + ((-1.0f) * ((fv2_data_->cur_dist - fv2_prev_dist_) / sampling_time2_)) + lv_data_->cur_vel;
    }
    else{  //All trucks' velocity sensors are fail
      crc_mode_ = 2;
      fv2_data_->crc_mode = crc_mode_;
    }
  }
}

void CentralRC::statusCheck(ZmqData *lv_data, ZmqData *fv1_data, ZmqData *fv2_data){
  uint8_t lv_mode = lv_data->lrc_mode;
  uint8_t fv1_mode = fv1_data->lrc_mode;
  uint8_t fv2_mode = fv2_data->lrc_mode;

  if ((lv_mode == 0) && (fv1_mode == 0) && (fv2_mode == 0)){
    crc_mode_ = 0;
  }
  else if (((lv_mode == 2) || (fv1_mode == 2) || (fv2_mode == 2)) || ((lv_mode == 1) && (fv1_mode == 1) && (fv2_mode == 1))){
    crc_mode_ = 2;
    //crc_mode_ = 1;
  }
  else{
    crc_mode_ = 1;
  }

  if (fv1_data->beta && fv1_data->gamma){
    lv_data->send_rear_camera_image = true;
  }
  else lv_data->send_rear_camera_image = false;

  if (fv2_data->beta && fv2_data->gamma){
    fv1_data->send_rear_camera_image = true;
  }
  else fv1_data->send_rear_camera_image = false;

}

float CentralRC::lowPassFilter(float sampling_time, float pred_vel){
  float res = 0.f;
  res = (tau_*prev_res_ + sampling_time*pred_vel)/(tau_+sampling_time);
  prev_res_ = res;
  return res;
}

void CentralRC::recordData(struct timeval *startTime){
  struct timeval currentTime;
  char file_name[] = "CRC_log00.csv";
  static char file[128] = {0x00, };
  char buf[512] = {0x00,};
  static bool flag = false;
  std::ifstream read_file;
  std::ofstream write_file;
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[7] = i/10 + '0';  //ASCII
      file_name[8] = i%10 + '0';
      sprintf(file, "%s%s", log_path_.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    write_file << "Time,Tar_vel0,Ref_vel0,Cur_vel0,Tar_dist0,Cur_dist0,Tar_vel1,Ref_vel1,Cur_vel1,Tar_dist1,Cur_dist1,Tar_vel2,Ref_vel2,Cur_vel2,Tar_dist2,Cur_dist2,Alpha0,Beta0,Gamma0,Alpha1,Beta1,Gamma1,Alpha2,Beta2,Gamma2,LRC_mode0,LRC_mode1,LRC_mode2,CRC_mode" << std::endl; //seconds
    flag = true;
 }
  else{
    std::scoped_lock lock(data_mutex_);
    gettimeofday(&currentTime, NULL);
    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
    sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", time_, lv_data_->tar_vel, lv_data_->ref_vel, lv_data_->cur_vel, lv_data_->tar_dist, lv_data_->cur_dist, fv1_data_->tar_vel, fv1_data_->ref_vel, fv1_data_->cur_vel, fv1_data_->tar_dist, fv1_data_->cur_dist, fv2_data_->tar_vel, fv2_data_->ref_vel, fv2_data_->cur_vel, fv2_data_->tar_dist, fv2_data_->cur_dist, lv_data_->alpha, lv_data_->beta, lv_data_->gamma, fv1_data_->alpha, fv1_data_->beta, fv1_data_->gamma, fv2_data_->alpha, fv2_data_->beta, fv2_data_->gamma, lv_data_->lrc_mode, fv1_data_->lrc_mode, fv2_data_->lrc_mode, crc_mode_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

void CentralRC::printStatus(){
  printf("\033[2J\033[1;1H");
  printf("CRC is running ...\n");
  printf("CRC and each MODEs of LV, FV1, FV2:\t%d || %d, %d, %d\n", crc_mode_, lv_data_->lrc_mode, fv1_data_->lrc_mode, fv2_data_->lrc_mode);
  printf("Predict Velocitys of LV, FV1, FV2:\t%.3f, %.3f, %.3f\n", lv_data_->est_vel, fv1_data_->est_vel, fv2_data_->est_vel);
  printf("Sampling_time_:\t%.6f, %.6f\n", sampling_time1_, sampling_time2_);
  printf("LV velocity:\t%.3f\n", lv_data_->cur_vel);
  printf("FV1 velocity:\t%.3f\n", fv1_data_->cur_vel);
  printf("FV2 velocity:\t%.3f\n", fv2_data_->cur_vel);
  printf("LV current distance:\t%.3f\n", lv_data_->cur_dist);
  printf("FV1 current distance:\t%.3f\n", fv1_data_->cur_dist);
  printf("FV2 current distance:\t%.3f\n", fv2_data_->cur_dist);
  printf("Size:\t%zu\n", sizeof(*lv_data_));
}

void CentralRC::updateData(ZmqData* zmq_data){
  std::scoped_lock lock(data_mutex_);
  if(zmq_data->tar_index == 30){
    if(zmq_data->src_index == 10){
      lv_data_->tar_vel = zmq_data->tar_vel;
      lv_data_->ref_vel = zmq_data->ref_vel;
      lv_data_->cur_vel = zmq_data->cur_vel;
      lv_data_->tar_dist = zmq_data->tar_dist;
      lv_data_->cur_dist = zmq_data->cur_dist;
      lv_data_->alpha = zmq_data->alpha;
      lv_data_->beta = zmq_data->beta;
      lv_data_->gamma = zmq_data->gamma;
      lv_data_->lrc_mode = zmq_data->lrc_mode;
    }
    else if(zmq_data->src_index == 11){
      fv1_data_->tar_vel = zmq_data->tar_vel;
      fv1_data_->ref_vel = zmq_data->ref_vel;
      fv1_data_->cur_vel = zmq_data->cur_vel;
      fv1_data_->preceding_truck_vel = lv_data_->cur_vel;
      fv1_data_->tar_dist = zmq_data->tar_dist;
      fv1_data_->cur_dist = zmq_data->cur_dist;
      fv1_data_->alpha = zmq_data->alpha;
      fv1_data_->beta = zmq_data->beta;
      fv1_data_->gamma = zmq_data->gamma;
      fv1_data_->lrc_mode = zmq_data->lrc_mode;
      getSamplingTime(fv1_data_->cur_dist, fv1_prev_dist_, 1);
    }
    else if(zmq_data->src_index == 12){
      fv2_data_->tar_vel = zmq_data->tar_vel;
      fv2_data_->ref_vel = zmq_data->ref_vel;
      fv2_data_->cur_vel = zmq_data->cur_vel;
      fv2_data_->preceding_truck_vel = fv1_data_->cur_vel;
      fv2_data_->tar_dist = zmq_data->tar_dist;
      fv2_data_->cur_dist = zmq_data->cur_dist;
      fv2_data_->alpha = zmq_data->alpha;
      fv2_data_->beta = zmq_data->beta;
      fv2_data_->gamma = zmq_data->gamma;
      fv2_data_->lrc_mode = zmq_data->lrc_mode;
      getSamplingTime(fv2_data_->cur_dist, fv2_prev_dist_, 2);
    }
  }
}

bool CentralRC::getSamplingTime(float cur_dist, float prev_dist, int idx){
  bool get_flag = false;
  if(!time_flag_){
    gettimeofday(&start_time1_, NULL);
    gettimeofday(&start_time2_, NULL);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    time_flag_ = true;
  }
  if(time_flag_ && (cur_dist != prev_dist) && idx == 1){
    gettimeofday(&end_time1_, NULL);
    sampling_time1_ = (end_time1_.tv_sec - start_time1_.tv_sec) + ((end_time1_.tv_usec - start_time1_.tv_usec)/1000000.0);  //seconds
    if (sampling_time1_ > 0.1f) sampling_time1_ = 0.1f;
    get_flag = true;
    gettimeofday(&start_time1_, NULL);
  }
  if(time_flag_ && (cur_dist != prev_dist) && idx == 2){
    gettimeofday(&end_time2_, NULL);
    sampling_time2_ = (end_time2_.tv_sec - start_time2_.tv_sec) + ((end_time2_.tv_usec - start_time2_.tv_usec)/1000000.0);  //seconds
    if (sampling_time2_ > 0.1f) sampling_time2_ = 0.1f;
    get_flag = true;
    gettimeofday(&start_time2_, NULL);
  }
  return get_flag;
}

void CentralRC::communicate(){  
  {
    std::scoped_lock lock(data_mutex_);
    statusCheck(lv_data_, fv1_data_, fv2_data_);
  }

  updateData(ZMQ_SOCKET_.rep_recv0_);
  updateData(ZMQ_SOCKET_.rep_recv1_);
  updateData(ZMQ_SOCKET_.rep_recv2_);

  estimateVelocity(10);
  estimateVelocity(11);
  estimateVelocity(12);
  
  recordData(&launch_time_);

  {
    std::scoped_lock lock(data_mutex_);
    fv1_prev_dist_ = fv1_data_->cur_dist;
    fv2_prev_dist_ = fv2_data_->cur_dist;
  }

  printStatus();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

}
