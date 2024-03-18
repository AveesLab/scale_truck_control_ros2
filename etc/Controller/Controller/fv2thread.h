#ifndef FV2THREAD_H
#define FV2THREAD_H

#include <QThread>

#include "zmq_class.h"

class Controller;

class FV2Thread : public QThread
{
    Q_OBJECT
public:
    explicit FV2Thread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(ZmqData zmq_data);
    void request(ZmqData zmq_data);
};

#endif // FV2THEAD_H
