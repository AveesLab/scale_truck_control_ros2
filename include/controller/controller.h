#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QMainWindow>
#include <QApplication>
#include <QMutex>
#include <QDebug>
#include <QLocale>
#include <QTranslator>
#include <QString>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

//C++
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <vector>
#include <sys/time.h>
#include <condition_variable>
#include <functional>
#include <memory>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "controller/lvthread.h"
#include "controller/fv1thread.h"
#include "controller/fv2thread.h"
#include "controller/ros2node.hpp"
#include "scale_truck_control_ros2/msg/cmd_data.hpp"

typedef scale_truck_control_ros2::msg::CmdData CmdData;

QT_BEGIN_NAMESPACE
namespace Ui { class Controller; }
QT_END_NAMESPACE

using namespace std::chrono_literals;

class Controller : public QMainWindow
{
    Q_OBJECT

public:
    Controller(const std::shared_ptr<Ros2Node>& ros2_node, QWidget *parent = nullptr);
    ~Controller();
    void sendData(int value_vel, int value_dist, int to);

    static QMutex lv_mutex_;
    static QMutex fv1_mutex_;
    static QMutex fv2_mutex_;

    static CmdData lv_data_;
    static CmdData fv1_data_;
    static CmdData fv2_data_;

    static int cnt;

    int MinVel;
    int MaxVel;
    int MinDist;
    int MaxDist;

private slots:
    void requestData(CmdData cmd_data);

    void updateData(CmdData cmd_data);

    void on_MVelSlider_valueChanged(int value);

    void on_MDistSlider_valueChanged(int value);

    void on_LVVelSlider_valueChanged(int value);

    void on_LVDistSlider_valueChanged(int value);

    void on_pushButton_clicked();

    void on_LVBox_activated(int index);

    void on_FV1Box_activated(int index);

    void on_FV2Box_activated(int index);

    void on_Send_clicked();

signals:
    void send(CmdData cmd_data);

private:
    Ui::Controller *ui;

    const std::shared_ptr<Ros2Node> ros2_node;

    //Publisher 
    rclcpp::Publisher<scale_truck_control_ros2::msg::CmdData>::SharedPtr XavPublisher_;

    //Subscriber 
    rclcpp::Subscription<scale_truck_control_ros2::msg::CmdData>::SharedPtr XavSubscriber_;

    //Callback Func
    void XavSubCallback(const scale_truck_control_ros2::msg::CmdData &msg);

    cv::Mat display_Map(CmdData cmd_data);
    LVThread* lv_thread_;
    FV1Thread* fv1_thread_;
    FV2Thread* fv2_thread_;

    struct timeval startTime_;
    double time_ = 0.0;
    double req_time_ = 0.0;
    std::string log_path_ = "/home/avees/logfiles/";
    void recordData(struct timeval *time);
};
#endif // CONTROLLER_H
