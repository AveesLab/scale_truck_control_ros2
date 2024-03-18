#include "lrc/lrc.hpp"

#define PATH "/home/avees/catkin_ws/logfiles/"

using namespace std;

namespace LocalResiliencyCoordinator{

LocalRC::LocalRC(ros::NodeHandle nh)
  : nodeHandle_(nh), ZMQ_SOCKET_(nh){
  
  init();  
}

LocalRC::~LocalRC(){
  is_node_running_ = false; 
  udpThread_.join();

  delete lrc_data_;
}

void LocalRC::init(){
  is_node_running_ = true;

  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string OcrSubTopicName;
  int OcrSubQueueSize;
  std::string XavPubTopicName;
  int XavPubQueueSize;
  std::string OcrPubTopicName;
  int OcrPubQueueSize;

  nodeHandle_.param("LrcParams/lrc_index", index_, 10);
  nodeHandle_.param("LrcParams/lrc_log_path", log_path_, std::string("/home/jetson/catkin_ws/logfiles/"));
  nodeHandle_.param("LrcParams/epsilon", epsilon_, 1.0f);
  nodeHandle_.param("LrcParams/lu_ob_A", a_, 0.6817f);
  nodeHandle_.param("LrcParams/lu_ob_B", b_, 0.3183f);
  nodeHandle_.param("LrcParams/lu_ob_L", l_, 0.2817f);
  nodeHandle_.param("LrcParams/rcm_vel", rcm_vel_, 0.6f);
  nodeHandle_.param("LrcParams/rcm_dist", rcm_dist_, 0.8f);
  nodeHandle_.param("LrcParams/enable_console_output", EnableConsoleOutput_, true);

  /******************************/
  /* ROS Topic Subscribe Option */
  /******************************/
  nodeHandle_.param("LrcSubPub/xavier_to_lrc/topic", XavSubTopicName, std::string("/xav2lrc_msg"));
  nodeHandle_.param("LrcSubPub/xavier_to_lrc/queue_size", XavSubQueueSize, 1);
  nodeHandle_.param("LrcSubPub/ocr_to_lrc/topic", OcrSubTopicName, std::string("/ocr2lrc_msg"));
  nodeHandle_.param("LrcSubPub/ocr_to_lrc/queue_size", OcrSubQueueSize, 1);

  /******************************/
  /* ROS Topic Publish Option */
  /******************************/
  nodeHandle_.param("LrcSubPub/lrc_to_xavier/topic", XavPubTopicName, std::string("/lrc2xav_msg"));
  nodeHandle_.param("LrcSubPub/lrc_to_xavier/queue_size", XavSubQueueSize, 1);
  nodeHandle_.param("LrcSubPub/lrc_to_ocr/topic", OcrPubTopicName, std::string("/lrc2ocr_msg"));
  nodeHandle_.param("LrcSubPub/lrc_to_ocr/queue_size", OcrPubQueueSize, 1);

  /************************/
  /* ROS Topic Subscriber */ 
  /************************/
  XavSubscriber_ = nodeHandle_.subscribe(XavSubTopicName, XavSubQueueSize, &LocalRC::XavCallback, this);
  OcrSubscriber_ = nodeHandle_.subscribe(OcrSubTopicName, OcrSubQueueSize, &LocalRC::OcrCallback, this);

  /************************/
  /* ROS Topic Publisher */ 
  /************************/
  XavPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc2xav>(XavPubTopicName, XavPubQueueSize);
  OcrPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc2ocr>(OcrPubTopicName, OcrPubQueueSize);

  lrc_mode_ = 0;
  crc_mode_ = 0;
  lrc_data_ = new ZmqData;
  lrc_data_->src_index = index_;
  lrc_data_->tar_index = 30;  //CRC

  lrcThread_ = std::thread(&LocalRC::communicate, this);
  tcpThread_ = std::thread(&LocalRC::request, this, lrc_data_);
  if (index_ == 10){
    udpThread_ = std::thread(&LocalRC::radio, this, lrc_data_);
  }
  else if (index_ == 11 || index_ == 12){
    udpThread_ = std::thread(&LocalRC::dish, this);
  }
}

bool LocalRC::isNodeRunning(){
  return is_node_running_;
}


void LocalRC::XavCallback(const scale_truck_control::xav2lrc &msg){
/* time delay check */
//  static struct timeval start_time, end_time;
//  static bool flag = false;
//  static double diff_time = 0.0;
//  double avg_time = 0.0;
//  static int cnt = 0;
//  if (!flag){
//    gettimeofday(&start_time, NULL);
//    flag = true;
//  }
//  else{
//    gettimeofday(&end_time, NULL);
//    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0 + (end_time.tv_usec - start_time.tv_usec) / 1000.0); 
//    cnt++;
//    avg_time = diff_time / cnt;
//    if (cnt > 3000){
//      diff_time = 0.0;
//      cnt = 0;
//    }
//    printf("2 - LRC sub from STC cycle time:\t %.3f\n", avg_time); 
//    gettimeofday(&start_time, NULL);
//  }

  std::scoped_lock lock(data_mutex_);
  angle_degree_ = msg.steer_angle;
  cur_dist_ = msg.cur_dist;
  if(index_ == 10){  //only LV LRC
    tar_dist_ = msg.tar_dist;
    tar_vel_ = msg.tar_vel;
  }
  fi_encoder_ = msg.fi_encoder;
  fi_camera_ = msg.fi_camera;
  fi_lidar_ = msg.fi_lidar;
  alpha_ = msg.alpha;
  beta_ = msg.beta;
  gamma_ = msg.gamma;
}

void LocalRC::OcrCallback(const scale_truck_control::ocr2lrc &msg){
/* time delay check */
//  static struct timeval start_time, end_time;
//  static bool flag = false;
//  static double diff_time = 0.0;
//  double avg_time = 0.0;
//  static int cnt = 0;
//  if (!flag){
//    gettimeofday(&start_time, NULL);
//    flag = true;
//  }
//  else{
//    gettimeofday(&end_time, NULL);
//    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0 + (end_time.tv_usec - start_time.tv_usec) / 1000.0); 
//    cnt++;
//    avg_time = diff_time / cnt;
//    if (cnt > 3000){
//      diff_time = 0.0;
//      cnt = 0;
//    }
//    printf("1 - LRC sub from OpenCR cycle time:\t %.3f\n", avg_time); 
//    gettimeofday(&start_time, NULL);
//  }

  std::scoped_lock lock(data_mutex_);
  ref_vel_ = msg.ref_vel;
  cur_vel_ = msg.cur_vel;
  sat_vel_ = msg.u_k;  //saturated velocity
}

void LocalRC::rosPub(){
  scale_truck_control::lrc2xav xav;
  scale_truck_control::lrc2ocr ocr;
  { 
    std::scoped_lock lock(data_mutex_);
    xav.cur_vel = cur_vel_;
    xav.tar_vel = tar_vel_;
    xav.tar_dist = tar_dist_;
    xav.alpha = alpha_;
    xav.send_rear_camera_image = send_rear_camera_image_;
    xav.lrc_mode = lrc_mode_;
    xav.crc_mode = crc_mode_;
    ocr.index = index_;
    ocr.steer_angle = angle_degree_;
    ocr.cur_dist = cur_dist_;
    ocr.tar_dist = tar_dist_;
    ocr.tar_vel = tar_vel_;
    ocr.est_vel = est_vel_;
    ocr.preceding_truck_vel = preceding_truck_vel_;
    ocr.fi_encoder = fi_encoder_;
    ocr.alpha = alpha_;
  }
  XavPublisher_.publish(xav);
  OcrPublisher_.publish(ocr);
}

void LocalRC::radio(ZmqData* zmq_data)
{
  while(isNodeRunning()){
    {
      std::scoped_lock lock(data_mutex_);
      zmq_data->tar_vel = tar_vel_;
      zmq_data->tar_dist = tar_dist_;
    }
    ZMQ_SOCKET_.radioZMQ(zmq_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LocalRC::dish()
{
  while(isNodeRunning()){
    ZMQ_SOCKET_.dishZMQ();
    updateData(ZMQ_SOCKET_.dsh_recv_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LocalRC::request(ZmqData* zmq_data){
  struct timeval startTime, endTime;
  double diff_time = 0.0;
  while(isNodeRunning()){
    {
      std::scoped_lock lock(data_mutex_);
      zmq_data->tar_vel = tar_vel_;
      zmq_data->ref_vel = ref_vel_;
      zmq_data->cur_vel = cur_vel_;
      zmq_data->tar_dist = tar_dist_;
      zmq_data->cur_dist = cur_dist_;
      zmq_data->alpha = alpha_;
      zmq_data->beta = beta_;
      zmq_data->gamma = gamma_;
      zmq_data->lrc_mode = lrc_mode_;
    }
    ZMQ_SOCKET_.requestZMQ(zmq_data);
    updateData(ZMQ_SOCKET_.req_recv_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LocalRC::encoderCheck(){
  std::scoped_lock lock(data_mutex_);
  if(!fi_encoder_){
//    hat_vel_ = a_ * hat_vel_ + b_ * sat_vel_ - l_ * (cur_vel_ - hat_vel_);
//    if(fabs(cur_vel_ - hat_vel_) > epsilon_){
//      alpha_ = true;
//    }
  }
  else{
    hat_vel_ = a_ * hat_vel_ + b_ * 2.0f - l_ * (0.0f - hat_vel_);
    if(fabs(0.0f - hat_vel_) > epsilon_){
      alpha_ = true;
    }
  }
}

void LocalRC::updateMode(uint8_t crc_mode){
  std::scoped_lock lock(data_mutex_);
  if(index_ == 10){  //LV
//    if(beta_ || gamma_){  //Camera sensor failure
//      lrc_mode_ = 2;  //GDM
//    }
//    else if(alpha_){
//      lrc_mode_ = 1;  //RCM
//    }
//    else{
      lrc_mode_ = 0;  //TM
//    }
  }
  else{  //FV1, FV2
    if(alpha_ && beta_ && gamma_){
      lrc_mode_ = 2;  
    }
    else if(alpha_ || beta_ || gamma_){
      lrc_mode_ = 1;  
    }
    else{
      lrc_mode_ = 0;
    }
  }
}

void LocalRC::updateData(ZmqData* zmq_data){
  std::scoped_lock lock(data_mutex_);
  if(zmq_data->src_index == 30){  //from CRC
    est_vel_ = zmq_data->est_vel;
    crc_mode_ = zmq_data->crc_mode;
    preceding_truck_vel_ = zmq_data->preceding_truck_vel;
    send_rear_camera_image_ = zmq_data->send_rear_camera_image;
  }
  else if(zmq_data->src_index == 10){  //from LV LRC to FVs LRC
    if (lrc_mode_ == 0) {
      tar_vel_ = zmq_data->tar_vel;
      tar_dist_ = zmq_data->tar_dist;
    }
    else {  //FV1 reference velocity and gap distance on RCM mode
      if(zmq_data->tar_vel > rcm_vel_) tar_vel_ = rcm_vel_;
      else tar_vel_ = zmq_data->tar_vel;
      if(zmq_data->tar_dist < rcm_dist_) tar_dist_ = rcm_dist_;
      else tar_dist_ = zmq_data->tar_dist;
    }
  }
}


void LocalRC::recordData(struct timeval *startTime){
  struct timeval currentTime;
  char file_name[] = "LRC_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  ifstream read_file;
  ofstream write_file;
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
    write_file << "Time,Tar_dist,Cur_dist,Tar_vel,Ref_vel,Cur_vel,Lrc_mode" << endl; //seconds
    flag = true;
  }
  else{
    std::scoped_lock lock(data_mutex_);
    gettimeofday(&currentTime, NULL);
    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
    sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%d", time_, tar_dist_, cur_dist_, tar_vel_, ref_vel_, cur_vel_, lrc_mode_);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << endl;
  }
  write_file.close();
}

void LocalRC::printStatus(){
  static int cnt = 0;
  static int CNT = 0;
  if (EnableConsoleOutput_){
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nPredict Velocity:\t%.3f", est_vel_);
    printf("\nTarget Velocity:\t%.3f", tar_vel_);
    printf("\nCurrent Velocity:\t%.3f", cur_vel_);
    printf("\nTarget Distance:\t%.3f", tar_dist_);
    printf("\nCurrent Distance:\t%.3f", cur_dist_);
    printf("\nEstimated Value:\t%.3f", fabs(cur_vel_ - hat_vel_));
    printf("\nalpha, beta, gamma:\t%d, %d, %d", alpha_, beta_, gamma_); 
    printf("\nMODE:\t%d", lrc_mode_);
    printf("\n");
  }
}

void LocalRC::communicate(){
  struct timeval startTime, endTime;
  gettimeofday(&startTime, NULL);
  static int cnt = 0;
  while(ros::ok()){
    //encoderCheck();
    updateMode(crc_mode_);
    rosPub();
    printStatus();

    //recordData(&startTime);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if(!isNodeRunning()){
      ros::requestShutdown();
      break;
    }
  }
}

}
