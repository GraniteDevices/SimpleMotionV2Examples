#include "mw.h"
#include "ui_mw.h"

MW::MW(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MW)
{
    ui->setupUi(this);


    //add two axis to QList
    Axis x, y;
    axis.append(x);
    axis.append(y);

    //start timer which polls axis statuses
    timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),SLOT(timerTick()));
    timer->start(100);//100ms
}

MW::~MW()
{
    delete ui;
}

void MW::on_connect_clicked()
{

}

void MW::on_disconnect_clicked()
{

}

void MW::on_sendSetpoints_clicked()
{
    axis[0].sendSetpoint(ui->newSetpointX->value());
    axis[1].sendSetpoint(ui->newSetpointY->value());
}

void MW::on_abortMotion_clicked()
{
    int i;
    for(i=0;i<axis.count();i++)
        axis[i].stopMotion();
}

void MW::on_clearFaults_clicked()
{
    int i;
    for(i=0;i<axis.count();i++)
        axis[i].clearFaults();
}

void MW::timerTick()
{
    //show timer counter in status bar to indicate that app is working
    static int tick=0;
    tick++;
    statusBar()->showMessage( QString("Update tick %1").arg(tick));

    //update statuses
    int i;
    for(i=0;i<axis.count();i++)
        axis[i].updateStatus();

    //update to UI
    ui->statusX->setText(Axis::statusToString( axis[0].getLastStatus() ));
    ui->statusY->setText(Axis::statusToString( axis[1].getLastStatus() ));

}
