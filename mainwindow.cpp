#include "mainwindow.h"
#include <QtSerialPort>
#include <QDebug>
#include <QLineEdit>
#include <QLayout>
#include <QTimer>
#include <QLabel>
#include <QPalette>
#include "drawing.h"
using namespace std;

int ind_p;
int bufShowSize=1000;
QwtPlot* d_plot;
myCurve* curveADC;
int ind_c;

QLineEdit* serial_le;
QLineEdit* V1_le;
QLineEdit* V2_le;
QLineEdit* t1_le;
QLineEdit* t2_le;
QLineEdit* dT_le;
QLineEdit* T_le;

vector<float> data_adc;

char buf[3];
QTimer timer;
int timer_cnt;
QString qstr;
QSerialPort port;
int serial_inited;


void drawingInit(QwtPlot* d_plot, QString title);

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    d_plot = new QwtPlot(this);
    drawingInit(d_plot,QString("ADC"));
    d_plot->setAxisScale(QwtPlot::yLeft,0,100);
    d_plot->setAxisScale(QwtPlot::xBottom,0,bufShowSize);
    curveADC=new myCurve(bufShowSize,data_adc,d_plot,"EMG",Qt::black,Qt::black,ind_c);


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

    QLabel* dT_label=new QLabel("dT");
    dT_label->setMaximumWidth(labels_width);

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

    serial_le=new QLineEdit("COM14");
    V1_le=new QLineEdit("100");
    V2_le=new QLineEdit("-50");
    t1_le=new QLineEdit("6");
    t2_le=new QLineEdit("12");
    dT_le=new QLineEdit("10");
    T_le=new QLineEdit("70");

    serial_le->setMaximumWidth(200);
    V1_le->setMaximumWidth(200);
    V2_le->setMaximumWidth(200);
    t1_le->setMaximumWidth(200);
    t2_le->setMaximumWidth(200);
    dT_le->setMaximumWidth(200);
    T_le->setMaximumWidth(200);

    //    COMInit();
    connect(serial_le,SIGNAL(returnPressed()),this,SLOT(COMInit()));
    //    serial_le=new QLineEdit("");
    auto* central = new QWidget;
    auto* lt=new QGridLayout;
    lt->addWidget(label,0,0,7,1);
    lt->addWidget(port_label,0,1);
    lt->addWidget(serial_le,0,2);

    lt->addWidget(V1_label,1,1);
    lt->addWidget(V1_le,1,2);

    lt->addWidget(V2_label,2,1);
    lt->addWidget(V2_le,2,2);

    lt->addWidget(t1_label,3,1);
    lt->addWidget(t1_le,3,2);

    lt->addWidget(t2_label,4,1);
    lt->addWidget(t2_le,4,2);

    lt->addWidget(dT_label,5,1);
    lt->addWidget(dT_le,5,2);

    lt->addWidget(T_label,6,1);
    lt->addWidget(T_le,6,2);
    lt->addWidget(d_plot,7,0,1,3);

    central->setLayout(lt);
    setCentralWidget(central);

    connect(V1_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(V2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(t1_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(t2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(dT_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(T_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));

    timer.setInterval(1);

    connect(&timer, SIGNAL(timeout()),this,SLOT(readADC()));
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
    if(port.open(QIODevice::ReadWrite))
    {

    }
    else
    {

    }

    serial_le->setDisabled(true);

    timer.start();
}

void MainWindow::readADC()
{
    int N=port.read(buf,30);
    for(int i=0;i<N;i++)
    {
        //    (uint8_t)buf[i];
        ind_c=(ind_c+1)%data_adc.size();
        //    ind_p=ind_c;

        data_adc[ind_c]=1000./(1+(uint8_t)buf[i]);//for -50
//        data_adc[ind_c]=(uint8_t)buf[i];
        curveADC->signalDrawing(0.019);
    }

//    qDebug()<<N;

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
    //        c=dT_le->text().toInt();
    //        port.write(&c,1);
    //        timer_cnt=0;
    //        timer.stop();
    //        break;

    //    }

    timer.setInterval(1);
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
        c=dT_le->text().toInt();
        port.write(&c,1);

        c=T_le->text().toInt();
        port.write(&c,1);
        //        timer_cnt=0;
        //        timer.stop();
        //        break;

    }

}


void drawingInit(QwtPlot* d_plot, QString title)
{

    //        setCentralWidget(MW);

    //canvas().resize(925,342)
    //    d_plot->canvas()->resize(100,150);
    //d_plot->autoRefresh();
    d_plot->setAutoReplot();
    //_______232

    // настройка функций
    QwtPlotPicker *d_picker =
            new QwtPlotPicker(
                QwtPlot::xBottom, QwtPlot::yLeft, // ассоциация с осями
                QwtPlotPicker::CrossRubberBand, // стиль перпендикулярных линий
                QwtPicker::ActiveOnly, // включение/выключение
                d_plot->canvas() ); // ассоциация с полем
    // Цвет перпендикулярных линий
    d_picker->setRubberBandPen( QColor( Qt::red ) );

    // цвет координат положения указателя
    d_picker->setTrackerPen( QColor( Qt::black ) );

    // непосредственное включение вышеописанных функций
    d_picker->setStateMachine( new QwtPickerDragPointMachine() );

    // Включить возможность приближения/удаления графика
    // #include <qwt_plot_magnifier.h>
    QwtPlotMagnifier *magnifier = new QwtPlotMagnifier(d_plot->canvas());
    // клавиша, активирующая приближение/удаление
    magnifier->setMouseButton(Qt::MidButton);
    // Включить возможность перемещения по графику
    // #include <qwt_plot_panner.h>
    QwtPlotPanner *d_panner = new QwtPlotPanner( d_plot->canvas() );
    // клавиша, активирующая перемещение
    d_panner->setMouseButton( Qt::RightButton );
    // Включить отображение координат курсора и двух перпендикулярных
    // линий в месте его отображения

    QwtText* qwtt=new QwtText(title);
    qwtt->setFont(QFont("Helvetica", 11,QFont::Normal));

    d_plot->setAxisScale(1,-500,500,200);
    d_plot->setTitle( *qwtt ); // заголовок
    d_plot->setCanvasBackground( Qt::white ); // цвет фона
    //    d_plot->set


    // Включить сетку
    // #include <qwt_plot_grid.h>
    QwtPlotGrid *grid = new QwtPlotGrid(); //

    grid->setMajorPen(QPen( Qt::gray, 2 )); // цвет линий и толщина
    grid->attach( d_plot ); // добавить сетку к полю графика


    d_plot->setMinimumSize(150,140);

}

MainWindow::~MainWindow()
{

}
