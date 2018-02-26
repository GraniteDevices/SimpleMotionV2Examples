#ifndef MW_H
#define MW_H

#include <QMainWindow>
#include <QList>
#include <QTimer>

#include "simplemotion.h"
#include "bufferedmotion.h"

namespace Ui {
class MW;
}

class MW : public QMainWindow
{
    Q_OBJECT

public:
    explicit MW(QWidget *parent = 0);
    ~MW();

private slots:
    void on_connect_clicked();

    void on_disconnect_clicked();

    void on_startMotion_clicked();

    void on_abortMotion_clicked();

    void on_clearFaults_clicked();

    void timerTick();

    void on_zero_clicked();

private:

    void syncClocks();
    static const int maxAxis=16;

    Ui::MW *ui;

    QTimer *timer;

    double streamTime;

    bool busOpen, motionActive;

    smbus bushandle;
    BufferedMotionAxis axis[maxAxis];

    void writeLog(QString msg);

    //return next point of motion stream
    QList <double> getNextTrajectoryCoordinates();

    //returns true on error. smDeviceErrors optional (contains detailed reason SM_ERR_PARAMETER)
    bool checkAndReportSMBusErrors(smint32 smDeviceErrors=0);

    //call this as often as possible, preferrably in own thread in loop, but here for simplicity we call it from timer (may slow down GUI)
    void feedDrives();

    //set buttons enabled/disabled depending on state
    void updateUIcontrols();
};

#endif // MW_H
