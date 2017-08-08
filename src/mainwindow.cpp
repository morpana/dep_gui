#include "dep_gui/mainwindow.h"
#include "ui_mainwindow.h"
#include <QtGlobal>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QObject::connect(ui->pushButton, SIGNAL(released()), this, SLOT(PrintStuff()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::PrintStuff(){
    QString s = ui->textEdit->toPlainText();
    std::cout << qPrintable(s);
}
