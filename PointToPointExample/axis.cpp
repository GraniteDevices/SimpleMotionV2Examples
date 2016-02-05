#include "axis.h"

Axis::Axis()
{
    address=0;
    status=NA;
    busHandle=-1;
}

void Axis::setBushandle(int handle)
{
    busHandle=handle;
}

//return true if axis is connected and accepts commands
bool Axis::isInUse()
{
    if(busHandle>=0 && address>0 )
        return true;
    else
        return false;
}


void Axis::setAddress(int addr)
{
    address=addr;
}

void Axis::updateStatus()
{
    //do nothing if axis is not acive ("Not in use" in spinbox)
    if(isInUse()==false)
    {
        status=NA;
        return;
    }
}

Axis::Status Axis::getLastStatus()
{
    return status;
}

void Axis::stopMotion()
{
    //do nothing if axis is not acive ("Not in use" in spinbox)
    if(isInUse()==false) return;

}

void Axis::sendSetpoint(int setp)
{
    //do nothing if axis is not acive ("Not in use" in spinbox)
    if(isInUse()==false) return;

    setpoint=setp;
}

void Axis::clearFaults()
{
    //do nothing if axis is not acive ("Not in use" in spinbox)
    if(isInUse()==false) return;

}

QString Axis::statusToString(const Status stat)
{
    switch(stat)
    {
    case NA: return "n/a"; break;
    case ConnectionError: return "Connection error"; break;
    case Busy: return "Busy"; break;
    case Ready: return "Ready"; break;
    case Faulted: return "Faulted"; break;
    default: return "undefined";break;
    }
}
