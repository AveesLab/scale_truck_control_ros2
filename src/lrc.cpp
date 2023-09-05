#include "lrc/lrc.hpp"

using namespace std::chrono_literals;

namespace LocalResiliencyCoordinator{

LocalRC::LocalRC(void) 
	: Node("LRC", rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
		      .automatically_declare_parameters_from_overrides(true))
{
  init();

}

LocalRC::~LocalRC(){
  isNodeRunning_ = false; 
}

void LocalRC::init(){

  isNodeRunning_ = true;

  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string OcrSubTopicName;
  int OcrSubQueueSize;

  std::string OcrPubTopicName;
  int OcrPubQueueSize;

  this->get_parameter_or("LrcParams/lrc_index", index_, 10);
  this->get_parameter_or("LrcParams/lrc_log_path", log_path_, std::string("/home/jetson/catkin_ws/logfiles/"));
  this->get_parameter_or("LrcParams/enable_console_output", EnableConsoleOutput_, true);
  /******************************/
  /* ROS Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("LrcSub/xavier_to_lrc/topic", XavSubTopicName, std::string("xav2lrc_msg"));
  this->get_parameter_or("LrcSub/xavier_to_lrc/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("LrcSub/ocr_to_lrc/topic", OcrSubTopicName, std::string("ocr2lrc_msg"));
  this->get_parameter_or("LrcSub/ocr_to_lrc/queue_size", OcrSubQueueSize, 1);

  /******************************/
  /* ROS Topic Publish Option */
  /******************************/
  this->get_parameter_or("LrcPub/lrc_to_ocr/topic", OcrPubTopicName, std::string("lrc2ocr_msg"));
  this->get_parameter_or("LrcPub/lrc_to_ocr/queue_size", OcrPubQueueSize, 1);

  /************************/
  /* ROS Topic Subscriber */
  /************************/
  XavSubscriber_ = this->create_subscription<ros2_msg::msg::Xav2lrc>(XavSubTopicName, XavSubQueueSize, std::bind(&LocalRC::XavCallback, this, std::placeholders::_1));

  OcrSubscriber_ = this->create_subscription<ros2_msg::msg::Ocr2lrc>(OcrSubTopicName, OcrSubQueueSize, std::bind(&LocalRC::OcrCallback, this, std::placeholders::_1));

  /************************/
  /* ROS Topic Publisher */
  /************************/
  OcrPublisher_ = this->create_publisher<ros2_msg::msg::Lrc2ocr>(OcrPubTopicName, OcrPubQueueSize);

  /*********************/
  /* spin & udp thread */
  /*********************/
  lrcThread_ = std::thread(&LocalRC::communicate, this);
}

bool LocalRC::isNodeRunning(){
  return isNodeRunning_;
}

void LocalRC::rosPub(){
  ros2_msg::msg::Lrc2ocr ocr;
  { 
    std::scoped_lock lock(data_mutex_);
    ocr.steer_angle = angle_degree_;
  }
  OcrPublisher_->publish(ocr);
}

//void LocalRC::recordData(struct timeval *startTime){
//  struct timeval currentTime;
//  char file_name[] = "LocalRC_log00.csv";
//  static char file[128] = {0x00, };
//  char buf[256] = {0x00,};
//  static bool flag = false;
//  std::ifstream read_file;
//  std::ofstream write_file;
//  if(!flag){
//    for(int i = 0; i < 100; i++){
//      file_name[7] = i/10 + '0';  //ASCII
//      file_name[8] = i%10 + '0';
//      sprintf(file, "%s%s", log_path_.c_str(), file_name);
//      read_file.open(file);
//      if(read_file.fail()){  //Check if the file exists
//        read_file.close();
//        write_file.open(file);
//        break;
//      }
//      read_file.close();
//    }
//    write_file << "Time,Tar_dist,Cur_dist,Tar_vel,Ref_vel,Cur_vel,Lrc_mode" << std::endl; //seconds
//    flag = true;
//  }
//  else{
//    std::scoped_lock lock(data_mutex_);
//    gettimeofday(&currentTime, NULL);
//    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
//    //sprintf(buf, "%.10e,%.3f,%.3f,%.3f,%.3f,%.3f,%d", time_, tar_dist_, cur_dist_, tar_vel_, ref_vel_, cur_vel_);
//    write_file.open(file, std::ios::out | std::ios::app);
//    write_file << buf << std::endl;
//  }
//  write_file.close();
//}

//void LocalRC::printStatus(){
//  if (EnableConsoleOutput_){
//    printf("\033[2J");
//    printf("\033[1;1H");
//    printf("LrcParams/lrc_index :%d\n", index_);
//    printf("Target Velocity:\t%.3f\n", tar_vel_);
//    printf("Current Velocity:\t%.3f\n", cur_vel_);
//    printf("Target Distance:\t%.3f\n", tar_dist_);
//    printf("Current Distance:\t%.3f\n", cur_dist_);
//    printf("ocr_vel:\t%.3f\n", ref_vel_);
//    printf("sat_vel:\t%.3f\n", sat_vel_);
////    printf("Tar|Cur Vel     : %3.3f | %3.3f m/s\n", tar_vel_, cur_vel_);
////    printf("Tar|Cur Dist    : %3.3f | %3.3f m\n", tar_dist_, cur_dist_);
//
//  }
//}

void LocalRC::communicate(){
  while(rclcpp::ok()){
    rosPub();
//    printStatus();

//    recordData(&startTime);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if(!isNodeRunning()){
      rclcpp::shutdown();
      break;
    }
  }
}

void LocalRC::XavCallback(const ros2_msg::msg::Xav2lrc::SharedPtr msg)
{
  std::scoped_lock lock(data_mutex_);
  angle_degree_ = msg->steer_angle;
}

void LocalRC::OcrCallback(const ros2_msg::msg::Ocr2lrc::SharedPtr msg)
{
  float ocr_angle = msg->cur_angle;
  float ocr_angle_output = msg->angle_output;

//  RCLCPP_INFO(this->get_logger(), "ocr_angle/output : %3.3f / %3.3f\n", ocr_angle, ocr_angle_output);

}

} /* namespace scale_truck_control */

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalResiliencyCoordinator::LocalRC>());
    rclcpp::shutdown();
    return 0;
}

