#ifndef FV2THREAD_H
#define FV2THREAD_H

#include <QThread>

#include "scale_truck_control_ros2/msg/cmd_data.hpp"
typedef scale_truck_control_ros2::msg::CmdData CmdData;

class Controller;

class FV2Thread : public QThread
{
    Q_OBJECT
public:
    explicit FV2Thread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(CmdData cmd_data);
    void request(CmdData cmd_data);
};

#endif // FV2THEAD_H
