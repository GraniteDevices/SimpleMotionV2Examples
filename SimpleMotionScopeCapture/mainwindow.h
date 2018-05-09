#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "simplemotion.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_capture_clicked();

private:
    Ui::MainWindow *ui;

    int nodeAddress;
    smbus handle;


    bool setupScope();
    SM_STATUS scopeWait();
    bool downloadScope( int nsamples, int *samples );
};

#endif // MAINWINDOW_H
