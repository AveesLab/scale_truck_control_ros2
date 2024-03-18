#ifndef FV1THREAD_H
#define FV1THREAD_H

#include <QThread>

#include "zmq_class.h"

class Controller;

class FV1Thread : public QThread
{
    Q_OBJECT
public:
    explicit FV1Thread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(ZmqData zmq_data);
    void request(ZmqData zmq_data);
};

#endif // FV1THREAD_H
