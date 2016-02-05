#ifndef AXIS_H
#define AXIS_H

#include <QString>
#include "simplemotion.h"

class Axis
{
public:
    enum Status {NA,ConnectionError,Ready,Busy,Faulted};

    Axis();

    void setBushandle(int handle);
    void setAddress(int addr);
    void updateStatus();
    Status getLastStatus();
    void stopMotion();
    void sendSetpoint(int setp);
    void clearFaults();

    static QString statusToString(const Status stat);

    //return true if axis is connected and accepts commands
    bool isInUse();

private:
    smbus busHandle;
    int address;
    Status status;
    int setpoint;
};

#endif // AXIS_H
