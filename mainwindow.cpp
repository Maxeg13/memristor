//v1 , v2 are positive for VAC_mode (convert to -v1 and v2)
#include "mainwindow.h"
#include <QtSerialPort>
#include <QDebug>
#include <QLineEdit>
#include <QLayout>
#include <QTimer>
#include <QPushButton>
#include <QLabel>
#include <QPalette>
#include "drawing.h"
using namespace std;
QPushButton *vac_btn, *filler_btn, *filler_btn1;

enum MODE
{
    CUSTOM,
    VAC,
    PROGRAM
};

MODE MD;

int ind_p;
int VAC_buf=300;//400
int bufShowSize=1000;
QwtPlot *d_plot, *set_plot;
myCurve* curveADC;
int ind_c;

QLineEdit* serial_le;
QLineEdit* V1_le;
QLineEdit* V2_le;
QLineEdit* t1_le;
QLineEdit* t2_le;
QLineEdit* dT_le;
QLineEdit* T_le;
QLineEdit* VAC_min_le;
QLineEdit* VAC_max_le;
float V1;

float mvs[]={296, 480, 1020, 1960, 3000};
float grid[]={10, 20 ,80, 100, 163};

float adc2mvs(float x)
{
    int a,b=-10;
    if(x<grid[0])
    {
        a=mvs[0]*(x/grid[0]);
    }
    else if(x<grid[1])
    {
        a=mvs[0]+(x-grid[0])*((mvs[1]-mvs[0])/(grid[1]-grid[0]));
        b=grid[0];
    }
    else if(x<grid[2])
    {
        a=mvs[1]+(x-grid[1])*((mvs[2]-mvs[1])/(grid[2]-grid[1]));
        b=grid[1];
    }
    else if(x<grid[3])
    {
        a=mvs[2]+(x-grid[2])*((mvs[3]-mvs[2])/(grid[3]-grid[2]));
        b=grid[2];
    }
    else
    {
        a=mvs[3]+(x-grid[3])*((mvs[4]-mvs[3])/(grid[4]-grid[3]));
        b=grid[3];
    }

    qDebug()<<b;
    return a;
}

vector<float> data_adc;

bool VAC_mode=0;
vector<float> current;
int current_ind;
vector<float> voltage;
int voltage_ind;
const int buf_N=30;
char buf[buf_N];
QTimer timer;
int timer_cnt;
QString qstr;
QSerialPort port;
int serial_inited;
myCurve *setCurve;


void drawingInit(QwtPlot* d_plot, QString title);

//void check(vector<float>& vv=vector<float>(3))
//{

//}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    voltage.resize(VAC_buf);
    current.resize(VAC_buf);
    //    check
    //    vector<float> vv=vector<float>(2);
    //    vv[3]=3;
    //    qDebug()<<vv.size();
    vac_btn=new QPushButton("no VAC mode");
    connect(vac_btn,SIGNAL(pressed()),this,SLOT(vac_btn_pressed()));
    filler_btn=new QPushButton();
    filler_btn1=new QPushButton();


    set_plot = new QwtPlot(this);
    set_plot->setAxisScale(1,-128,128);
    drawingInit(set_plot,"VA dependence");
    setCurve=new myCurve(set_plot,QString("VAC"), QColor(255,0,0,255));
    setCurve->setPen(QColor(255,0,0,255));
    QwtSymbol* symbol2 = new QwtSymbol( QwtSymbol::Ellipse,
                                        QBrush(QColor(0,0,0)), QPen( Qt::black, 2 ), QSize( 3, 3 ) );
    setCurve->setSymbol( symbol2 );

    vector<float> xx;
    vector<float> yy;
    xx.resize(30);
    yy.resize(30);
    int i;
    for(auto &iter: xx)
    {
        i++;
        iter=i;
    }
    i=0;
    for(auto &iter: yy)
    {
        i++;
        iter=i;
    }

    setCurve->set_Drawing(xx,yy,0,1);


    d_plot = new QwtPlot(this);
    drawingInit(d_plot,QString("current"));

    //y axis
    d_plot->setAxisScale(QwtPlot::yLeft,0,256);
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

    QLabel* VAC_min_label=new QLabel("VAC_min");
    VAC_min_label->setMaximumWidth(labels_width);

    QLabel* VAC_max_label=new QLabel("VAC_max");
    VAC_max_label->setMaximumWidth(labels_width);
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
    V1_le=new QLineEdit("10");
    V2_le=new QLineEdit("3");
    t1_le=new QLineEdit("3");
    t2_le=new QLineEdit("12");
    dT_le=new QLineEdit("10");
    T_le=new QLineEdit("60");
    VAC_min_le=new QLineEdit("126");
    VAC_max_le=new QLineEdit("126");

    serial_le->setMaximumWidth(200);
    V1_le->setMaximumWidth(200);
    V2_le->setMaximumWidth(200);
    t1_le->setMaximumWidth(200);
    t2_le->setMaximumWidth(200);
    dT_le->setMaximumWidth(200);
    T_le->setMaximumWidth(200);
    VAC_min_le->setMaximumWidth(200);
    VAC_max_le->setMaximumWidth(200);

    //    COMInit();
    connect(serial_le,SIGNAL(returnPressed()),this,SLOT(COMInit()));
    //    serial_le=new QLineEdit("");
    auto* central = new QWidget;
    auto* lt=new QGridLayout;
    lt->addWidget(label,0,0,9,1);
    label->setMaximumWidth(500);
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
    //    lt->addWidget(T_le,6,2);

    lt->addWidget(VAC_min_label,7,1);
    lt->addWidget(VAC_min_le,7,2);

    lt->addWidget(VAC_max_label,8,1);
    lt->addWidget(VAC_max_le,8,2);

    lt->addWidget(vac_btn,9,0);
    lt->addWidget(filler_btn,9,1,1,2);
    //    lt->addWidget(filler_btn1,7,2);


    lt->addWidget(d_plot,10,0,1,3);
    lt->addWidget(set_plot,11,0,1,3);
    //    set_plot->setMaximumWidth(500);


    central->setLayout(lt);
    setCentralWidget(central);

    connect(V1_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(V2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(t1_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(t2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(dT_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(T_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(VAC_min_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(VAC_max_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));


    timer.setInterval(1);


    connect(&timer, SIGNAL(timeout()),this,SLOT(UDP_get()));
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

void MainWindow::UDP_get()
{

    static uint8_t ptr=0;
    uint8_t buf1;
    uint16_t h;
    int a,b;
    int N=port.read(buf,buf_N);
    for(int i=0;i<N;i++)
    {
        if(!VAC_mode)
        {
            //    (uint8_t)buf[i];
            ind_c=(ind_c+1)%data_adc.size();
            //    ind_p=ind_c;
            //a=buf[i];
            //b=buf1;
            //            data_adc[ind_c]=((a<<8)+(b));//for -50  25500
            //qDebug()<<a;
            //qDebug()<<b;
            //qDebug()<<'\n';

            data_adc[ind_c]=256-(.01+(uint8_t)buf[i]);
            //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
            curveADC->signalDrawing(1);


            //        buf1=buf[i];


        }
        else//VAC
        {
            //             qDebug()<<(uint8_t)buf[i];
            switch(ptr)
            {

            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    //                    qDebug()<<"hello";
                    ptr=2;
                    ptr%=3;
                }
                break;
                //            case 1:
                //                h=((uint16_t)(uint8_t)buf[i]);

                //                break;
            case 1:

                //                qDebug()<<buf1;
                //                buf1=(uint8_t)buf[i];
//                current[current_ind]=(((((uint16_t)buf[i])<<8)|buf1));
                current[current_ind]=-(uint8_t)buf[i];
                current_ind++;
                current_ind%=current.size();


                break;
                //            case 2:
                //                qDebug()<<(uint16_t)buf[i];
                //                current[current_ind]=(((((uint16_t)buf[i])<<8)|buf1));
                //                current_ind++;
                //                current_ind%=current.size();
                //                break;

            case 2:
                //                data_adc[ind_c]=(.01+(uint8_t)buf[i]);
                ind_c=(ind_c+1)%data_adc.size();
                //                curveADC->signalDrawing(1);
                voltage[voltage_ind]=(int8_t)buf[i];
                voltage_ind++;
                voltage_ind%=voltage.size();
                setCurve->set_Drawing(voltage,current,voltage_ind,current_ind);
                break;


            }
            ptr++;
            ptr%=3;


        }
    }
    //        qDebug()<<current_ind<<voltage_ind;
    //    qDebug()<<N;

}

void MainWindow::vac_btn_pressed()
{
    VAC_mode=!VAC_mode;
    char c;
    oneSend();
    if(VAC_mode)
    {
        data_adc.resize(data_adc.size(),0);
        for (auto& it:data_adc)
            it=0;

        curveADC->signalDrawing(1);

        vac_btn->setText("VAC mode");


    }
    else
    {
        vac_btn->setText("no VAC mode");

    }

}


void MainWindow::oneSend()
{



    timer.setInterval(1);
    char c;
    // switch(timer_cnt)
    {
        c=255;
        port.write(&c,1);
        //    case 0:

        c=VAC_mode;
        port.write(&c,1);


        //        V1=V1_le->text().toFloat();
        //        V2=V2_le->text().toFloat();

        if(VAC_mode)
            c=VAC_min_le->text().toInt();
        else
            c=V1_le->text().toInt();
        port.write(&c,1);
        timer_cnt++;
        //        break;
        //    case 1:
        if(VAC_mode)
            c=VAC_max_le->text().toInt();
        else
            c=V2_le->text().toInt();
        //        qDebug()<<VAC_max_le->text().toInt();
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
