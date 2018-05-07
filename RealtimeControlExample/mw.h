#ifndef MW_H
#define MW_H

#include <QMainWindow>
#include "communicationthread.h"

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

    void on_decrement10000_clicked();

    void on_decrement1000_clicked();

    void on_decrement100_clicked();

    void on_decrement10_clicked();

    void on_increment10_clicked();

    void on_increment100_clicked();

    void on_increment1000_clicked();

    void on_increment10000_clicked();

    void on_resetLocalTrackingError_clicked();

    void on_resetDriveFaults_clicked();

    void on_updateRate_valueChanged(int arg1);

    void on_positionGain_valueChanged(double arg1);

    void on_maxSpeed_valueChanged(int value);

    void on_trackingErrorTolerance_valueChanged(int arg1);

    void logMessage(QString text);

    void updateStatus(int posSetpoint, int posFeedback, int velSetpoint );

private:
    Ui::MW *ui;

    void applySettigns();

    CommunicationThread commThread;
};

#endif // MW_H
