#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QMainWindow>
#include <QMutex>
#include <QDebug>

#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>

#include "lvthread.h"
#include "fv1thread.h"
#include "fv2thread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Controller; }
QT_END_NAMESPACE

class Controller : public QMainWindow
{
    Q_OBJECT

public:
    Controller(QWidget *parent = nullptr);
    ~Controller();
    void sendData(int value_vel, int value_dist, int to);

    static QMutex lv_mutex_;
    static QMutex fv1_mutex_;
    static QMutex fv2_mutex_;

    static ZmqData lv_data_;
    static ZmqData fv1_data_;
    static ZmqData fv2_data_;

    static int cnt;

    int MinVel;
    int MaxVel;
    int MinDist;
    int MaxDist;
    int LV_fi_encoder;
    int FV1_fi_encoder;
    int FV1_fi_camera;
    int FV1_fi_lidar;
    int FV2_fi_encoder;
    int FV2_fi_camera;
    int FV2_fi_lidar;

private slots:
    void requestData(ZmqData zmq_data);

    void updateData(ZmqData zmq_data);

    void on_MVelSlider_valueChanged(int value);

    void on_MDistSlider_valueChanged(int value);

    void on_LVVelSlider_valueChanged(int value);

    void on_LVDistSlider_valueChanged(int value);

    //void on_FV1VelSlider_valueChanged(int value);

    //void on_FV1DistSlider_valueChanged(int value);

    //void on_FV2VelSlider_valueChanged(int value);

    //void on_FV2DistSlider_valueChanged(int value);

    void on_pushButton_clicked();

    void on_LVBox_activated(int index);

    void on_FV1Box_activated(int index);

    void on_FV2Box_activated(int index);

    void on_Send_clicked();

    void on_FV1_alpha_toggled(bool checked);

    void on_FV1_beta_toggled(bool checked);

    void on_FV1_gamma_toggled(bool checked);

    void on_FV2_alpha_toggled(bool checked);

    void on_FV2_beta_toggled(bool checked);

    void on_FV2_gamma_toggled(bool checked);

    void on_LV_alpha_toggled(bool checked);

signals:
    void send(ZmqData zmq_data);

private:
    Ui::Controller *ui;
    ZMQ_CLASS ZMQ_SOCKET_;
    cv::Mat display_Map(ZmqData zmq_data);
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
