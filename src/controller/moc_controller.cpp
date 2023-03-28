/****************************************************************************
** Meta object code from reading C++ file 'controller.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "controller.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'controller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Controller_t {
    QByteArrayData data[26];
    char stringdata0[430];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Controller_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Controller_t qt_meta_stringdata_Controller = {
    {
QT_MOC_LITERAL(0, 0, 10), // "Controller"
QT_MOC_LITERAL(1, 11, 4), // "send"
QT_MOC_LITERAL(2, 16, 0), // ""
QT_MOC_LITERAL(3, 17, 7), // "ZmqData"
QT_MOC_LITERAL(4, 25, 8), // "zmq_data"
QT_MOC_LITERAL(5, 34, 11), // "requestData"
QT_MOC_LITERAL(6, 46, 10), // "updateData"
QT_MOC_LITERAL(7, 57, 26), // "on_MVelSlider_valueChanged"
QT_MOC_LITERAL(8, 84, 5), // "value"
QT_MOC_LITERAL(9, 90, 27), // "on_MDistSlider_valueChanged"
QT_MOC_LITERAL(10, 118, 27), // "on_LVVelSlider_valueChanged"
QT_MOC_LITERAL(11, 146, 28), // "on_LVDistSlider_valueChanged"
QT_MOC_LITERAL(12, 175, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(13, 197, 18), // "on_LVBox_activated"
QT_MOC_LITERAL(14, 216, 5), // "index"
QT_MOC_LITERAL(15, 222, 19), // "on_FV1Box_activated"
QT_MOC_LITERAL(16, 242, 19), // "on_FV2Box_activated"
QT_MOC_LITERAL(17, 262, 15), // "on_Send_clicked"
QT_MOC_LITERAL(18, 278, 20), // "on_FV1_alpha_toggled"
QT_MOC_LITERAL(19, 299, 7), // "checked"
QT_MOC_LITERAL(20, 307, 19), // "on_FV1_beta_toggled"
QT_MOC_LITERAL(21, 327, 20), // "on_FV1_gamma_toggled"
QT_MOC_LITERAL(22, 348, 20), // "on_FV2_alpha_toggled"
QT_MOC_LITERAL(23, 369, 19), // "on_FV2_beta_toggled"
QT_MOC_LITERAL(24, 389, 20), // "on_FV2_gamma_toggled"
QT_MOC_LITERAL(25, 410, 19) // "on_LV_alpha_toggled"

    },
    "Controller\0send\0\0ZmqData\0zmq_data\0"
    "requestData\0updateData\0"
    "on_MVelSlider_valueChanged\0value\0"
    "on_MDistSlider_valueChanged\0"
    "on_LVVelSlider_valueChanged\0"
    "on_LVDistSlider_valueChanged\0"
    "on_pushButton_clicked\0on_LVBox_activated\0"
    "index\0on_FV1Box_activated\0on_FV2Box_activated\0"
    "on_Send_clicked\0on_FV1_alpha_toggled\0"
    "checked\0on_FV1_beta_toggled\0"
    "on_FV1_gamma_toggled\0on_FV2_alpha_toggled\0"
    "on_FV2_beta_toggled\0on_FV2_gamma_toggled\0"
    "on_LV_alpha_toggled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Controller[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  109,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,  112,    2, 0x08 /* Private */,
       6,    1,  115,    2, 0x08 /* Private */,
       7,    1,  118,    2, 0x08 /* Private */,
       9,    1,  121,    2, 0x08 /* Private */,
      10,    1,  124,    2, 0x08 /* Private */,
      11,    1,  127,    2, 0x08 /* Private */,
      12,    0,  130,    2, 0x08 /* Private */,
      13,    1,  131,    2, 0x08 /* Private */,
      15,    1,  134,    2, 0x08 /* Private */,
      16,    1,  137,    2, 0x08 /* Private */,
      17,    0,  140,    2, 0x08 /* Private */,
      18,    1,  141,    2, 0x08 /* Private */,
      20,    1,  144,    2, 0x08 /* Private */,
      21,    1,  147,    2, 0x08 /* Private */,
      22,    1,  150,    2, 0x08 /* Private */,
      23,    1,  153,    2, 0x08 /* Private */,
      24,    1,  156,    2, 0x08 /* Private */,
      25,    1,  159,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   19,
    QMetaType::Void, QMetaType::Bool,   19,
    QMetaType::Void, QMetaType::Bool,   19,
    QMetaType::Void, QMetaType::Bool,   19,
    QMetaType::Void, QMetaType::Bool,   19,
    QMetaType::Void, QMetaType::Bool,   19,
    QMetaType::Void, QMetaType::Bool,   19,

       0        // eod
};

void Controller::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Controller *_t = static_cast<Controller *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->send((*reinterpret_cast< ZmqData(*)>(_a[1]))); break;
        case 1: _t->requestData((*reinterpret_cast< ZmqData(*)>(_a[1]))); break;
        case 2: _t->updateData((*reinterpret_cast< ZmqData(*)>(_a[1]))); break;
        case 3: _t->on_MVelSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->on_MDistSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->on_LVVelSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_LVDistSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_pushButton_clicked(); break;
        case 8: _t->on_LVBox_activated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_FV1Box_activated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_FV2Box_activated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_Send_clicked(); break;
        case 12: _t->on_FV1_alpha_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 13: _t->on_FV1_beta_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: _t->on_FV1_gamma_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 15: _t->on_FV2_alpha_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->on_FV2_beta_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 17: _t->on_FV2_gamma_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->on_LV_alpha_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (Controller::*_t)(ZmqData );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Controller::send)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject Controller::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_Controller.data,
      qt_meta_data_Controller,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Controller::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Controller::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Controller.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int Controller::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 19)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 19;
    }
    return _id;
}

// SIGNAL 0
void Controller::send(ZmqData _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
