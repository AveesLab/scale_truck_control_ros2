#include "controller.h"
#include "ui_controller.h"

QMutex Controller::lv_mutex_;
QMutex Controller::fv1_mutex_;
QMutex Controller::fv2_mutex_;

ZmqData Controller::lv_data_;
ZmqData Controller::fv1_data_;
ZmqData Controller::fv2_data_;

Controller::Controller(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Controller), ZMQ_SOCKET_()
{
    ui->setupUi(this);
    lv_thread_ = new LVThread(this);
    fv1_thread_ = new FV1Thread(this);
    fv2_thread_ = new FV2Thread(this);
    qRegisterMetaType<ZmqData>("ZmqData");
    connect(lv_thread_, SIGNAL(request(ZmqData)), this, SLOT(requestData(ZmqData)), Qt::DirectConnection);
    connect(lv_thread_, SIGNAL(setValue(ZmqData)),this,SLOT(updateData(ZmqData)), Qt::AutoConnection);
    connect(fv1_thread_, SIGNAL(request(ZmqData)), this, SLOT(requestData(ZmqData)), Qt::DirectConnection);
    connect(fv1_thread_, SIGNAL(setValue(ZmqData)),this,SLOT(updateData(ZmqData)), Qt::AutoConnection);
    connect(fv2_thread_, SIGNAL(request(ZmqData)), this, SLOT(requestData(ZmqData)), Qt::DirectConnection);
    connect(fv2_thread_, SIGNAL(setValue(ZmqData)),this,SLOT(updateData(ZmqData)), Qt::AutoConnection);

    if (ZMQ_SOCKET_.req_flag0_) lv_thread_->start();
    if (ZMQ_SOCKET_.req_flag1_) fv1_thread_->start();
    if (ZMQ_SOCKET_.req_flag2_) fv2_thread_->start();

    gettimeofday(&startTime_, NULL);

    int DefaultVel = 0; // cm/s
    MinVel = 0; // cm/s
    MaxVel = 140; // cm/s
    int DefaultDist = 80; // cm
    MinDist = 15; // cm
    MaxDist = 1000; // cm

    LV_fi_encoder = 0;
    FV1_fi_encoder = 0;
    FV1_fi_camera = 0;
    FV1_fi_lidar = 0;
    FV2_fi_encoder = 0;
    FV2_fi_camera = 0;
    FV2_fi_lidar = 0;

    ui->LV_beta->setEnabled(false);
    ui->LV_gamma->setEnabled(false);

    /* Setup Velocity Slider */
    ui->MVelSlider->setMaximum(MaxVel);
    ui->LVVelSlider->setMaximum(MaxVel);
    ui->FV1VelSlider->setMaximum(MaxVel);
    ui->FV2VelSlider->setMaximum(MaxVel);
    ui->MVelSlider->setMinimum(MinVel);
    ui->LVVelSlider->setMinimum(MinVel);
    ui->FV1VelSlider->setMinimum(MinVel);
    ui->FV2VelSlider->setMinimum(MinVel);
    ui->MVelSlider->setValue(DefaultVel);
    ui->LVVelSlider->setValue(DefaultVel);
    ui->FV1VelSlider->setValue(DefaultVel);
    ui->FV2VelSlider->setValue(DefaultVel);

    /* Setup Distance Slider */
    ui->MDistSlider->setMaximum(MaxDist);
    ui->LVDistSlider->setMaximum(MaxDist);
    ui->FV1DistSlider->setMaximum(MaxDist);
    ui->FV2DistSlider->setMaximum(MaxDist);
    ui->MDistSlider->setMinimum(MinDist);
    ui->LVDistSlider->setMinimum(MinDist);
    ui->FV1DistSlider->setMinimum(MinDist);
    ui->FV2DistSlider->setMinimum(MinDist);
    ui->MDistSlider->setValue(DefaultDist);
    ui->LVDistSlider->setValue(DefaultDist);
    ui->FV1DistSlider->setValue(DefaultDist);
    ui->FV2DistSlider->setValue(DefaultDist);

    /* Setup Velocity & Distance Bar */
    ui->LVVelBar->setMaximum(MaxVel);
    ui->FV1VelBar->setMaximum(MaxVel);
    ui->FV2VelBar->setMaximum(MaxVel);
    ui->LVVelBar->setMinimum(MinVel);
    ui->FV1VelBar->setMinimum(MinVel);
    ui->FV2VelBar->setMinimum(MinVel);
    ui->LVDistBar->setMaximum(MaxDist);
    ui->FV1DistBar->setMaximum(MaxDist);
    ui->FV2DistBar->setMaximum(MaxDist);
    ui->LVDistBar->setMinimum(MinDist);
    ui->FV1DistBar->setMinimum(MinDist);
    ui->FV2DistBar->setMinimum(MinDist);

    /* Setup Mode */
    ui->LVBox->setCurrentIndex(1);
    ui->FV1Box->setCurrentIndex(2);
    ui->FV2Box->setCurrentIndex(3);

    ZmqData default_data;
    float default_dist = DefaultDist/100.0f;
    default_data.src_index = 20;
    default_data.tar_index = 0;
    default_data.tar_vel = DefaultVel;
    default_data.tar_dist = default_dist;
    requestData(default_data);
}

void Controller::recordData(struct timeval *startTime){
  struct timeval currentTime;
  char file_name[] = "CC_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  std::ifstream read_file;
  std::ofstream write_file;
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[6] = i/10 + '0';  //ASCI
      file_name[7] = i%10 + '0';
      sprintf(file, "%s%s", log_path_.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    //write_file << "Time[s],Predict,Target,Current,Saturation,Estimate,Alpha" << endl;
    write_file << "Time,Request_time,Cur_dist" << std::endl; //seconds
    flag = true;
  }
  else{
    lv_mutex_.lock();
    gettimeofday(&currentTime, NULL);
    time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
    //sprintf(buf, "%.3e,%.3f,%.3f,%.3f,%.3f,%.3f,%d", time_, est_vel_, tar_vel_, cur_vel_, sat_vel_, fabs(cur_vel_ - hat_vel_), alpha_);
    sprintf(buf, "%.3e,%.3f,%.3f", time_, req_time_, ZMQ_SOCKET_.req_recv0_->cur_dist);
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
    lv_mutex_.unlock();
  }
  write_file.close();
}

void Controller::requestData(ZmqData zmq_data)
{
    ZmqData send_data = zmq_data;
    struct timeval startTime, endTime;

    if (send_data.tar_index == 0){
        lv_mutex_.lock();
        gettimeofday(&startTime, NULL);
        ZMQ_SOCKET_.requestZMQ(&send_data);
        gettimeofday(&endTime, NULL);
        req_time_ = ((endTime.tv_sec - startTime.tv_sec)* 1000.0) + ((endTime.tv_usec - startTime.tv_usec)/1000.0);
        lv_data_ = *ZMQ_SOCKET_.req_recv0_;
        lv_mutex_.unlock();
        //recordData(&startTime_);
    }
    else if (send_data.tar_index == 1){
        fv1_mutex_.lock();
        ZMQ_SOCKET_.requestZMQ(&send_data);
        fv1_data_ = *ZMQ_SOCKET_.req_recv1_;
        fv1_mutex_.unlock();
    }
    else if (send_data.tar_index == 2){
        fv2_mutex_.lock();
        ZMQ_SOCKET_.requestZMQ(&send_data);
        fv2_data_ = *ZMQ_SOCKET_.req_recv2_;
        fv2_mutex_.unlock();
    }
}

void Controller::updateData(ZmqData zmq_data)
{
    ZmqData tmp;
    int vel, dist;
    float deci = 100.0f;
    tmp = zmq_data;
    vel = tmp.cur_vel*deci;
    dist = tmp.cur_dist*deci;

    if(tmp.tar_index == 20){
      if(tmp.src_index == 0)
      {
          ui->LVCurVel->setText(QString::number(vel/deci));
          if(vel > MaxVel) {
              vel = MaxVel;
          }
          ui->LVVelBar->setValue(vel);
          ui->LVCurDist->setText(QString::number(dist/deci));
          if(dist > MaxDist) {
              dist = MaxDist;
          }
          ui->LVDistBar->setValue(dist);
          cv::Mat frame;
          display_Map(tmp).copyTo(frame);
          ui->LV_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));
      }
      else if (tmp.src_index == 1)
      {
          ui->FV1CurVel->setText(QString::number(vel/deci));
          if(vel > MaxVel) {
            vel = MaxVel;
          }
          ui->FV1VelBar->setValue(vel);
          ui->FV1CurDist->setText(QString::number(dist/deci));
          if(dist > MaxDist) {
            dist = MaxDist;
          }
          ui->FV1DistBar->setValue(dist);
          cv::Mat frame;
          display_Map(tmp).copyTo(frame);
          ui->FV1_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));
      }
      else if (tmp.src_index == 2)
      {
          ui->FV2CurVel->setText(QString::number(vel/deci));
          if(vel > MaxVel) {
            vel = MaxVel;
          }
          ui->FV2VelBar->setValue(vel);
          ui->FV2CurDist->setText(QString::number(dist/deci));
          if(dist > MaxDist) {
            dist = MaxDist;
          }
          ui->FV2DistBar->setValue(dist);
          cv::Mat frame;
          display_Map(tmp).copyTo(frame);
          ui->FV2_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));
      }
    }
}

cv::Mat Controller::display_Map(ZmqData value)
{
    int width = 350;
    int height = 350;
    cv::Mat map_frame = cv::Mat::zeros(cv::Size(width,height), CV_8UC3);
    int check_dist = 50;
    for(int i=0;i<height;i+=check_dist)
        cv::line(map_frame, cv::Point(0,i), cv::Point(width,i),cv::Scalar::all(100));
    for(int i=0;i<width;i+=check_dist)
        cv::line(map_frame, cv::Point(i,0), cv::Point(i,height),cv::Scalar::all(100));

    int centerY = height-100, centerX=width/2;
    cv::rectangle(map_frame, cv::Rect(cv::Point(centerX-9, centerY+50), cv::Point(centerX+9, centerY)), cv::Scalar(20,20,255), -1);

    std::vector<cv::Point> RpointList;
    std::vector<cv::Point> LpointList;
    std::vector<cv::Point> CpointList;
    int cam_height = 480, cam_width = 640; // pixel
    float cam_y_dist = 98, cam_x_dist = 77; // cm
    for(int i = -550; i < cam_height*1.2; i++){
        cv::Point temp_point;
        temp_point.y = i*cam_y_dist/cam_height-26;
        temp_point.y += (centerY-cam_y_dist);
        temp_point.x = (value.coef[0].a*pow(i,2) + value.coef[0].b*i + value.coef[0].c)*cam_x_dist/cam_width;
        temp_point.x += centerX-cam_x_dist/2;
        RpointList.push_back(temp_point);
        temp_point.x = (value.coef[1].a*pow(i,2) + value.coef[1].b*i + value.coef[1].c)*cam_x_dist/cam_width;
        temp_point.x += centerX-cam_x_dist/2;
        LpointList.push_back(temp_point);
        temp_point.x = (value.coef[2].a*pow(i,2) + value.coef[2].b*i + value.coef[2].c)*cam_x_dist/cam_width;
        temp_point.x += centerX-cam_x_dist/2;
        CpointList.push_back(temp_point);
    }
    const cv::Point* right_points_point = (const cv::Point*) cv::Mat(RpointList).data;
    int right_points_number = cv::Mat(RpointList).rows;
    const cv::Point* left_points_point = (const cv::Point*) cv::Mat(LpointList).data;
    int left_points_number = cv::Mat(LpointList).rows;
    const cv::Point* center_points_point = (const cv::Point*) cv::Mat(CpointList).data;
    int center_points_number = cv::Mat(CpointList).rows;

    cv::polylines(map_frame, &right_points_point, &right_points_number, 1, false, cv::Scalar::all(255), 2);
    cv::polylines(map_frame, &left_points_point, &left_points_number, 1, false, cv::Scalar::all(255), 2);
    cv::polylines(map_frame, &center_points_point, &center_points_number, 1, false, cv::Scalar(200,255,200), 2);

    float r = value.cur_dist*100; // cm
    float theta = value.cur_angle*M_PI/180; // rad
    if(r < 250) {
      int X = r*sin(theta)+centerX;
      int Y = -r*cos(theta)+centerY;
      cv::circle(map_frame,cv::Point(X,Y), 5, cv::Scalar(50,50,255), -1);
    }

    //cv::line(map_frame, cv::Point(width/2 - check_dist,124 + value.roi_dist*100/490),cv::Point(width/2 + check_dist,124+value.roi_dist*100/490), cv::Scalar(255,100,100),3);

    cv::Mat swap_frame;
    cv::cvtColor(map_frame, swap_frame, cv::COLOR_BGR2RGB);
    return swap_frame;
}

Controller::~Controller()
{
    delete ui;
}


void Controller::on_MVelSlider_valueChanged(int value)
{
    ui->MTarVel->setText(QString::number(value/100.0)); // m/s
    ui->LVVelSlider->setValue(value);
    ui->FV1VelSlider->setValue(value);
    ui->FV2VelSlider->setValue(value);
}

void Controller::on_MDistSlider_valueChanged(int value)
{
    ui->MTarDist->setText(QString::number(value/100.0)); // m
    ui->LVDistSlider->setValue(value);
    ui->FV1DistSlider->setValue(value);
    ui->FV2DistSlider->setValue(value);
}

void Controller::on_LVVelSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    float tar_vel = 0.0f, tar_dist = 0.0f;
    ZmqData zmq_data;

    value_vel = value;
    value_dist = ui->LVDistSlider->value();

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 0;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = LV_fi_encoder;
    requestData(zmq_data);

    ui->LVVelSlider->setValue(value_vel);
    ui->FV1VelSlider->setValue(value_vel);
    ui->FV2VelSlider->setValue(value_vel);
    ui->LVTarVel->setText(QString::number(value/100.0)); // m/s
    ui->FV1TarVel->setText(QString::number(value/100.0)); // m/s
    ui->FV2TarVel->setText(QString::number(value/100.0)); // m/s
}

void Controller::on_LVDistSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    float tar_vel = 0.0f, tar_dist = 0.0f;
    ZmqData zmq_data;

    value_vel = ui->LVVelSlider->value();
    value_dist = value;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 0;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = LV_fi_encoder;
    requestData(zmq_data);

    ui->LVDistSlider->setValue(value_dist);
    ui->FV1DistSlider->setValue(value_dist);
    ui->FV2DistSlider->setValue(value_dist);
    ui->LVTarDist->setText(QString::number(value/100.0)); // m
    ui->FV1TarDist->setText(QString::number(value/100.0)); // m
    ui->FV2TarDist->setText(QString::number(value/100.0)); // m
}
/*
void Controller::on_FV1VelSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    float tar_vel = 0.0f, tar_dist = 0.0f;
    ZmqData zmq_data;

    value_vel = value;
    value_dist = ui->FV1DistSlider->value();

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 1;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV1_fi_encoder;
    zmq_data.fi_camera = FV1_fi_camera;
    zmq_data.fi_lidar = FV1_fi_lidar;
    requestData(zmq_data);

    ui->FV1TarVel->setText(QString::number(value/100.0)); // m/s
}

void Controller::on_FV1DistSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    float tar_vel = 0.0f, tar_dist = 0.0f;
    ZmqData zmq_data;

    value_vel = ui->FV1VelSlider->value();
    value_dist = value;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 1;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV1_fi_encoder;
    zmq_data.fi_camera = FV1_fi_camera;
    zmq_data.fi_lidar = FV1_fi_lidar;
    requestData(zmq_data);

    ui->FV1TarDist->setText(QString::number(value/100.0)); // m
}

void Controller::on_FV2VelSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    float tar_vel = 0.0f, tar_dist = 0.0f;
    ZmqData zmq_data;

    value_vel = value;
    value_dist = ui->FV2DistSlider->value();

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 2;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV2_fi_encoder;
    zmq_data.fi_camera = FV2_fi_camera;
    zmq_data.fi_lidar = FV2_lidar;
    requestData(zmq_data);

    ui->FV2TarVel->setText(QString::number(value/100.0)); // m/s
}

void Controller::on_FV2DistSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    float tar_vel = 0.0f, tar_dist = 0.0f;
    ZmqData zmq_data;

    value_vel = ui->FV2VelSlider->value();
    value_dist = value;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 2;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV2_fi_encoder;
    zmq_data.fi_camera = FV2_fi_camera;
    zmq_data.fi_lidar = FV2_lidar;
    requestData(zmq_data);

    ui->FV2TarDist->setText(QString::number(value/100.0)); // m
}
*/
void Controller::on_pushButton_clicked()  //Emergency stop
{
    ZmqData zmq_data;
    float LV_dist = ui->LVDistSlider->value()/100.0f;
    ui->MVelSlider->setValue(0);
    ui->LVVelSlider->setValue(0);
    ui->FV1VelSlider->setValue(0);
    ui->FV2VelSlider->setValue(0);

    zmq_data.src_index = 20;
    zmq_data.tar_index = 0;
    zmq_data.tar_vel = 0;
    zmq_data.tar_dist = LV_dist;
    zmq_data.fi_encoder = LV_fi_encoder;
    requestData(zmq_data);
}

void Controller::on_LVBox_activated(int index)
{
    if(index == 0)
    {
        ui->LVVelSlider->setEnabled(true);
        ui->LVDistSlider->setEnabled(true);
    }
    else if(index == 1)
    {
        ui->LVVelSlider->setEnabled(true);
        ui->LVDistSlider->setEnabled(true);
    }
    else if(index == 2)
    {
        ui->LVVelSlider->setEnabled(false);
        ui->LVDistSlider->setEnabled(true);
    }
    else if(index == 3)
    {
        ui->LVVelSlider->setEnabled(false);
        ui->LVDistSlider->setEnabled(true);
    }
}


void Controller::on_FV1Box_activated(int index)
{
    if(index == 0)
    {
        ui->FV1VelSlider->setEnabled(true);
        ui->FV1DistSlider->setEnabled(true);
    }
    else if(index == 1)
    {
        ui->FV1VelSlider->setEnabled(true);
        ui->FV1DistSlider->setEnabled(true);
    }
    else if(index == 2)
    {
        ui->FV1VelSlider->setEnabled(false);
        ui->FV1DistSlider->setEnabled(true);
    }
    else if(index == 3)
    {
        ui->FV1VelSlider->setEnabled(false);
        ui->FV1DistSlider->setEnabled(true);
    }
}


void Controller::on_FV2Box_activated(int index)
{
    if(index == 0)
    {
        ui->FV2VelSlider->setEnabled(true);
        ui->FV2DistSlider->setEnabled(true);
    }
    else if(index == 1)
    {
        ui->FV2VelSlider->setEnabled(true);
        ui->FV2DistSlider->setEnabled(true);
    }
    else if(index == 2)
    {
        ui->FV2VelSlider->setEnabled(false);
        ui->FV2DistSlider->setEnabled(true);
    }
    else if(index == 3)
    {
        ui->FV2VelSlider->setEnabled(false);
        ui->FV2DistSlider->setEnabled(true);
    }
}

void Controller::on_Send_clicked()
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int vel = ui->MTarVel->text().split(" ")[0].toFloat()*100;
    ui->MVelSlider->setValue(vel);
    ui->LVVelSlider->setValue(vel);
    ui->FV1VelSlider->setValue(vel);
    ui->FV2VelSlider->setValue(vel);
    int dist = ui->MTarDist->text().split(" ")[0].toFloat()*100;
    ui->MDistSlider->setValue(dist);
    ui->LVDistSlider->setValue(dist);
    ui->FV1DistSlider->setValue(dist);
    ui->FV2DistSlider->setValue(dist);

    if(ui->LVVelSlider->value() >= 10) {
      tar_vel = ui->LVVelSlider->value()/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = ui->LVDistSlider->value()/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 0;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = LV_fi_encoder;

    requestData(zmq_data);
}

void Controller::on_LV_alpha_toggled(bool checked)
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int value_vel = ui->LVVelSlider->value();
    int value_dist = ui->LVDistSlider->value();
    LV_fi_encoder = checked;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 0;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = LV_fi_encoder;
    requestData(zmq_data);
}

void Controller::on_FV1_alpha_toggled(bool checked)
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int value_vel = ui->FV1VelSlider->value();
    int value_dist = ui->FV1DistSlider->value();
    FV1_fi_encoder = checked;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 1;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV1_fi_encoder;
    zmq_data.fi_camera = FV1_fi_camera;
    zmq_data.fi_lidar = FV1_fi_lidar;
    requestData(zmq_data);
}


void Controller::on_FV1_beta_toggled(bool checked)
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int value_vel = ui->FV1VelSlider->value();
    int value_dist = ui->FV1DistSlider->value();
    FV1_fi_camera = checked;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 1;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV1_fi_encoder;
    zmq_data.fi_camera = FV1_fi_camera;
    zmq_data.fi_lidar = FV1_fi_lidar;
    requestData(zmq_data);
}


void Controller::on_FV1_gamma_toggled(bool checked)
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int value_vel = ui->FV1VelSlider->value();
    int value_dist = ui->FV1DistSlider->value();
    FV1_fi_lidar = checked;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 1;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV1_fi_encoder;
    zmq_data.fi_camera = FV1_fi_camera;
    zmq_data.fi_lidar = FV1_fi_lidar;
    requestData(zmq_data);
}


void Controller::on_FV2_alpha_toggled(bool checked)
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int value_vel = ui->FV2VelSlider->value();
    int value_dist = ui->FV2DistSlider->value();
    FV2_fi_encoder = checked;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 2;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV2_fi_encoder;
    zmq_data.fi_camera = FV2_fi_camera;
    zmq_data.fi_lidar = FV2_fi_lidar;
    requestData(zmq_data);
}


void Controller::on_FV2_beta_toggled(bool checked)
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int value_vel = ui->FV2VelSlider->value();
    int value_dist = ui->FV2DistSlider->value();
    FV2_fi_camera = checked;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 2;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV2_fi_encoder;
    zmq_data.fi_camera = FV2_fi_camera;
    zmq_data.fi_lidar = FV2_fi_lidar;
    requestData(zmq_data);
}


void Controller::on_FV2_gamma_toggled(bool checked)
{
    ZmqData zmq_data;
    float tar_vel, tar_dist;

    int value_vel = ui->FV2VelSlider->value();
    int value_dist = ui->FV2DistSlider->value();
    FV2_fi_lidar = checked;

    if(value_vel >= 10) {
      tar_vel = value_vel/100.0f;
    }
    else {
      tar_vel = 0;
    }
    tar_dist = value_dist/100.0f;
    zmq_data.src_index = 20;
    zmq_data.tar_index = 2;
    zmq_data.tar_vel = tar_vel;
    zmq_data.tar_dist = tar_dist;
    zmq_data.fi_encoder = FV2_fi_encoder;
    zmq_data.fi_camera = FV2_fi_camera;
    zmq_data.fi_lidar = FV2_fi_lidar;
    requestData(zmq_data);
}
