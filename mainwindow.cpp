#include "mainwindow.h"
#include <QtSerialPort>
#include <QLineEdit>
#include <QLayout>
#include <QTimer>
#include <QLabel>
#include <QPalette>

QLineEdit* serial_le;
QLineEdit* V1_le;
QLineEdit* V2_le;
QLineEdit* t1_le;
QLineEdit* t2_le;
QLineEdit* T_le;
QTimer timer;
int timer_cnt;
QString qstr;
QSerialPort port;
int serial_inited;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    //    auto* pixmapScheme=new QPixmap("arrow-left.png");

    QImage image("C:/Users/chibi/Documents/memristor1/Scheme.png");
    QImage image2 = image.scaled(250, 200);

    //    QPalette palette;
    //    QPixmap pic("C:/Users/chibi/Documents/memristor1/Scheme.png");
    //pic.res
    QLabel *label = new QLabel(this);
    label->setScaledContents(true);
    label->setPixmap(QPixmap::fromImage(image2));

    int labels_width=60;
    QLabel* port_label=new QLabel("COM:");
    port_label->setMaximumWidth(labels_width);
    QLabel* V1_label=new QLabel("V1");
    V1_label->setMaximumWidth(labels_width);
    QLabel* V2_label=new QLabel("V2");
    V2_label->setMaximumWidth(labels_width);

    QLabel* t1_label=new QLabel("tau1");
    t1_label->setMaximumWidth(labels_width);
    QLabel* t2_label=new QLabel("tau2");
    t2_label->setMaximumWidth(labels_width);

    QLabel* T_label=new QLabel("T");
    T_label->setMaximumWidth(labels_width);
    //    palette.setBrush(QPalette::Window, QBrush(scaled));
    //    QWidget *w= new QWidget(this);
    //    w->setGeometry(0,0,800,480);
    //    w->show();
    //    w->setPalette(palette);

    //    QPalette Pal();
    //    // устанавливаем цвет фона
    //    Pal.setColor(QPalette::Background, Qt::white);
    //    setAutoFillBackground(true);
    //    setPalette(Pal);
    //    show();
    setStyleSheet("background-color: white");


    timer.setInterval(10);
//    connect(&timer, SIGNAL(timeout()),this,SLOT(oneSend()));
    serial_le=new QLineEdit("COM14");
    V1_le=new QLineEdit("100");
    V2_le=new QLineEdit("-50");
    t1_le=new QLineEdit("6");
    t2_le=new QLineEdit("12");
    T_le=new QLineEdit("10");
//    COMInit();
    connect(serial_le,SIGNAL(returnPressed()),this,SLOT(COMInit()));
    //    serial_le=new QLineEdit("");
    auto* central = new QWidget;
    auto* lt=new QGridLayout;
    lt->addWidget(label,0,0,1,2);
    lt->addWidget(port_label,1,0);
    lt->addWidget(serial_le,1,1);

    lt->addWidget(V1_label,2,0);
    lt->addWidget(V1_le,2,1);

    lt->addWidget(V2_label,3,0);
    lt->addWidget(V2_le,3,1);

    lt->addWidget(t1_label,4,0);
    lt->addWidget(t1_le,4,1);

    lt->addWidget(t2_label,5,0);
    lt->addWidget(t2_le,5,1);

    lt->addWidget(T_label,6,0);
    lt->addWidget(T_le,6,1);
    //    lt->addWidget(V2_le);
    central->setLayout(lt);
    setCentralWidget(central);

    connect(V1_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(V2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(t1_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(t2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(T_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
}


void MainWindow::COMInit()
{
    serial_inited=1;
    qstr=serial_le->text();
    std::string str1=qstr.toUtf8().constData();
    std::wstring str(str1.begin(),str1.end());
    port.setPortName(qstr);
    port.setBaudRate(9600);
    port.setStopBits(QSerialPort::TwoStop);
    if(port.open(QIODevice::WriteOnly))
    {

    }
    else
    {

    }

    serial_le->setDisabled(true);
}

void MainWindow::oneSend()
{

//    char c;
//    switch(timer_cnt)
//    {
//    case 0:
//        c=V1_le->text().toInt();
//        port.write(&c,1);
//        timer_cnt++;
//        break;
//    case 1:
//        c=V2_le->text().toInt();
//        port.write(&c,1);
//        timer_cnt++;
//        break;
//    case 2:
//        c=t1_le->text().toInt();
//        port.write(&c,1);
//        timer_cnt++;
//        break;
//    case 3:
//        c=t2_le->text().toInt();
//        port.write(&c,1);
//        timer_cnt++;
//        break;
//    case 4:
//        c=T_le->text().toInt();
//        port.write(&c,1);
//        timer_cnt=0;
//        timer.stop();
//        break;

//    }


    char c;
   // switch(timer_cnt)
    {
//    case 0:
        c=V1_le->text().toInt();
        port.write(&c,1);
        timer_cnt++;
//        break;
//    case 1:
        c=V2_le->text().toInt();
        port.write(&c,1);
        timer_cnt++;
//        break;
//    case 2:
        c=t1_le->text().toInt();
        port.write(&c,1);
        timer_cnt++;
//        break;
//    case 3:
        c=t2_le->text().toInt();
        port.write(&c,1);
//        timer_cnt++;
//        break;
//    case 4:
        c=T_le->text().toInt();
        port.write(&c,1);
        timer_cnt=0;
        timer.stop();
//        break;

    }

}

MainWindow::~MainWindow()
{

}
