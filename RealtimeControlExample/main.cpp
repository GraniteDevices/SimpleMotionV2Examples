#include <QApplication>
#include "mw.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MW w;
    w.show();

    return a.exec();
}

