#ifndef LVTHREAD_H
#define LVTHREAD_H

#include <QThread>

#include "scale_truck_control_ros2/msg/cmd_data.hpp"
typedef scale_truck_control_ros2::msg::CmdData CmdData;

class Controller;

class LVThread : public QThread
{
    Q_OBJECT
public:
    explicit LVThread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(CmdData cmd_data);
    void request(CmdData cmd_data);
};

#endif // LVTHREAD_H
