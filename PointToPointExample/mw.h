#ifndef MW_H
#define MW_H

#include <QMainWindow>
#include <QList>
#include <QTimer>
#include "axis.h"
#include "simplemotion.h"

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

    void on_sendSetpoints_clicked();

    void on_abortMotion_clicked();

    void on_clearFaults_clicked();

    void timerTick();

private:
    Ui::MW *ui;
    QList <Axis> axis;
    QTimer *timer;

    smbus busHandle;
};

#endif // MW_H
