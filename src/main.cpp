#include <QtGui>
#include "dep_gui/mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    std::thread publish_combination = w.spawn_thread();
    w.show();

    return a.exec();
}
