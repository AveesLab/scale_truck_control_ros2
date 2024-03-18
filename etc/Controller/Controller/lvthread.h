#ifndef LVTHREAD_H
#define LVTHREAD_H

#include <QThread>

#include "zmq_class.h"

class Controller;

class LVThread : public QThread
{
    Q_OBJECT
public:
    explicit LVThread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(ZmqData zmq_data);
    void request(ZmqData zmq_data);
};

#endif // LVTHREAD_H
