#ifndef FV1THREAD_H
#define FV1THREAD_H

#include <QThread>

#include "ros2_msg/msg/cmd_data.hpp"
typedef ros2_msg::msg::CmdData CmdData;

class Controller;

class FV1Thread : public QThread
{
    Q_OBJECT
public:
    explicit FV1Thread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(CmdData cmd_data);
    void request(CmdData cmd_data);
};

#endif // FV1THREAD_H
