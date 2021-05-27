//v1 , v2 are positive for VAC_mode (convert to -v1 and v2)
//20 us to 3.5 ms , 1V to 5V

//
#include "mainwindow.h"
#include <QtSerialPort>
#include <QComboBox>
#include <QCheckBox>
#include <QDebug>
#include <QLineEdit>
#include <QSlider>
#include <QLayout>
#include <QTimer>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QPalette>
#include "drawing.h"
#include <QFile>
#include <QTextStream>
#include <QKeyEvent>
#include <QMovie>
#include <map>
//#include <pair>

bool imitation_on =
        false;
//                true;
float V_koef=0.070;
float I_koef=11.;
using namespace std;
QPushButton *vac_btn,*custom_btn, *prog_btn, *filler_btn, *analyze_btn,
*filler_btn1, *rest_btn, *gather_mult_btn, *separ_mult_btn, *shots_btn;

enum MODE
{
    CUSTOM,
    VAC,
    PROGRAM,
    GATHER_MULT,
    SEPAR_MULT,
    ONE_SHOT,
    ANALYZE
};

MODE MD;

map<int,int> mapIV;
int reversed[8]={0,0,0,0,
                 0,0,0,0};
int adc_shift=120;
int ind_p;
int chan=0;
int VAC_buf=300;//400
int bufShowSize=40;
QCheckBox* VAC_check;
QCheckBox* write_check;
QwtPlot *cur_plot, *set_plot;
myCurve* curveADC;
QwtPlotCurve  *curveADC2;
//QwtPlot* ;
int ind_c;
int ind_c1;
int ind_c2;
QLabel* V_set_label;
QLabel* V_pl_max_label;
QLineEdit* serial_le;
QSlider* V_pl_max_slider;
QSlider* V_set_slider;
QSlider* V_ref_slider;
QSlider* VAC_mini_slider;
//QLineEdit* V2_le;
QLabel* targ_label;
QSlider* V_reset_slider;
QLineEdit* t2_le;
QLineEdit* dT_le;
QLineEdit* T_le;
QLabel* VAC_min_label;
QLabel* VAC_max_label;
QSlider* targ_slider;
QSlider* VAC_min_slider;
int V_m_=25;
int VAC_min[8]={V_m_,V_m_,V_m_,V_m_,
                V_m_,V_m_,V_m_,V_m_};
QSlider* VAC_max_slider;
int VAC_max[8]={V_m_,V_m_,V_m_,V_m_,
                V_m_,V_m_,V_m_,V_m_};
//QLineEdit* chan_le;
QComboBox* chan_cb;
QCheckBox* reverse_check;
QLabel* reset_label;
QLabel* t2_label;

//QLabel* t2_label;
uint8_t PROGRAM_done=0;
float V1;

float mvs[]={296, 480, 1020, 1960, 3000};
float grid[]={10, 20 ,80, 100, 163};

uint8_t ptr=0;
QLabel *imit_label;
vector<float> data_adc;
vector<float> data_adc1;
vector<float> data_adc2;
QLabel* V_ref_label;
vector<float> current;
int current_ind;
vector<float> voltage;
int voltage_ind;
const int buf_N=30;
char buf[buf_N];
QTimer serial_get_timer;
QTimer imit_timer;
QTimer one_shot_timer;
//int ;
QString qstr;
QSerialPort port;
int serial_inited;
myCurve *setCurve;
//QString filename = "Data.txt";

void drawingInit(QwtPlot* d_plot, QString title);
void WriteFile(QString s);

void ReadFile(QString s, map<int,int>& m);


QFile* file_rand_stat;
QFile* file_analyze;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    mapIV.erase(mapIV.begin(),mapIV.end());
    curveADC2=new QwtPlotCurve;

//    file_rand_stat = new QFile(QString("C:\\Users\\DrPepper\\Documents\\memristor\\rand_stat.txt"));
     file_rand_stat = new QFile(QString("rand_stat.txt"));

    file_analyze = new QFile(QString("C:\\Users\\DrPepper\\Documents\\memristor\\analyze.txt"));
    file_analyze->open(QIODevice::WriteOnly);


//    ReadFile("ShortCircuit", mapIV);
    //qDebug()<<QString("1:34").split(":")[1];
    voltage.resize(VAC_buf);
    current.resize(VAC_buf);
    //    check
    //    vector<float> vv=vector<float>(2);
    //    vv[3]=3;
    //    qDebug()<<vv.size();

    rest_btn= new QPushButton("take a rest");
    prog_btn=new QPushButton("PROGRAM MODE");
    custom_btn=new QPushButton("CUSTOM MODE");
    gather_mult_btn = new QPushButton("gather mult");
    separ_mult_btn = new QPushButton("separ mult");
    shots_btn = new QPushButton("SHOT LOOP");
    analyze_btn = new QPushButton("ANALYZE");
    vac_btn=new QPushButton("VAC MODE");
    VAC_check=new QCheckBox("check: ");
    write_check = new QCheckBox("write on");

    //////
    /// \brief connect
    ///
    ///
    connect(write_check, SIGNAL(stateChanged(int)), this, SLOT(write_check_state_changed(int)));
    connect(shots_btn,SIGNAL(pressed()),this,SLOT(shots_btn_pressed()));
    connect(separ_mult_btn,SIGNAL(pressed()),this,SLOT(separ_mult_btn_pressed()));
    connect(gather_mult_btn,SIGNAL(pressed()),this,SLOT(gather_mult_btn_pressed()));
    connect(rest_btn,SIGNAL(pressed()),this,SLOT(rest_btn_pressed()));
    connect(vac_btn,SIGNAL(pressed()),this,SLOT(vac_btn_pressed()));
    connect(analyze_btn,SIGNAL(pressed()),this,SLOT(analyze_btn_pressed()));
    connect(custom_btn,SIGNAL(pressed()),this,SLOT(custom_btn_pressed()));
    //    connect(vac_btn,SIGNAL(pressed()),this,SLOT(vac_btn_pressed()));
    connect(prog_btn,SIGNAL(pressed()),this,SLOT(prog_btn_pressed()));
    filler_btn=new QPushButton();
    filler_btn1=new QPushButton();


    set_plot = new QwtPlot(this);
    set_plot->setAxisScale(1,-128,128);
    set_plot->setAxisTitle(0,QString("I, mkA"));
    set_plot->setAxisTitle(2,QString("Voltage, V"));
    //    set_plot->enableAxis(0);
    //    set_plot->enableAxis(1);
    drawingInit(set_plot,"VA dependence");
    setCurve=new myCurve(set_plot,QString("VAC"), QColor(255,0,0,255));
    setCurve->setPen(QColor(255,0,0,255));
    QwtSymbol* symbol2 = new QwtSymbol( QwtSymbol::Ellipse,
                                        QBrush(QColor(0,0,0)), QPen( Qt::black, 2 ), QSize( 2, 2 ) );
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


    cur_plot = new QwtPlot(this);
    drawingInit(cur_plot,QString("current"));

    //y axis
    cur_plot->setAxisTitle(0,QString("I, mkA"));
    cur_plot->setAxisTitle(2,QString("time"));
    //    cur_plot->setAxisScale(QwtPlot::yLeft,-512*I_koef,512*I_koef);
    cur_plot->setAxisScale(QwtPlot::xBottom,0,bufShowSize);
    curveADC=new myCurve(bufShowSize,data_adc,cur_plot,"EMG",Qt::black,Qt::black,ind_c);
    data_adc1.resize(bufShowSize);
    ind_c1=0;
    data_adc2.resize(bufShowSize);
    ind_c2=0;
    //    QImage image("C:/Users/DrPepper/Documents/memristor/Scheme.png");
    //    QMovie *movie = new QMovie("./rezero.gif");
    QImage image("./Crossbar.jpg");
    QImage image2 = image.scaled(250, 200);

    //    QPalette palette;
    //    QPixmap pic("C:/Users/chibi/Documents/memristor1/Scheme.png");
    //pic.res
    QLabel *picture_label = new QLabel(this);
    picture_label->setScaledContents(true);
    //    label->setMovie(movie);
    //    movie->start();
    picture_label->setPixmap(QPixmap::fromImage(image2));


    QImage imit_img("E:\\pics\\graph1.png");
    imit_label = new QLabel(this);
    imit_label->setScaledContents(true);
    //    label->setMovie(movie);
    //    movie->start();
    imit_label->setPixmap(QPixmap::fromImage(imit_img));


    int labels_width=80;
    QLabel* port_label=new QLabel("COM:");
    port_label->setMaximumWidth(labels_width);
    V_set_label=new QLabel("V set");
    V_set_label->setMaximumWidth(labels_width);
    V_ref_label=new QLabel("V2 (ref)");
    V_ref_label->setMaximumWidth(labels_width);

    reset_label=new QLabel("V reset: ");
    reset_label->setMaximumWidth(labels_width);
    t2_label=new QLabel("tau2");
    t2_label->setMaximumWidth(labels_width);

    targ_label=new QLabel("targ");
    V_pl_max_label= new QLabel("V+max: ");

    QLabel* dT_label=new QLabel("dT");
    dT_label->setMaximumWidth(labels_width);

    QLabel* T_label=new QLabel("T");
    T_label->setMaximumWidth(labels_width);

    VAC_min_label=new QLabel("VAC-");
    VAC_min_label->setMaximumWidth(labels_width);

    VAC_max_label=new QLabel("VAC+");
    VAC_max_label->setMaximumWidth(labels_width);

    QLabel* chan_label = new QLabel("cnannel ind");
    chan_label->setMaximumWidth(labels_width);

    QLabel* reverse_label = new QLabel("is reversed");
    reverse_label->setMaximumWidth(labels_width);
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



    //common parameterization
    V_reset_slider=new QSlider(Qt::Orientation::Horizontal);
    VAC_mini_slider = new QSlider(Qt::Orientation::Horizontal);
    V_pl_max_slider = new QSlider(Qt::Orientation::Horizontal);
    targ_slider = new QSlider(Qt::Orientation::Horizontal);
    V_set_slider = new QSlider(Qt::Orientation::Horizontal);
    V_ref_slider=new QSlider(Qt::Orientation::Horizontal);
    VAC_max_slider=new QSlider(Qt::Orientation::Horizontal);
    VAC_min_slider=new QSlider(Qt::Orientation::Horizontal);
    vector<QSlider*> sliders={V_reset_slider, VAC_mini_slider, V_pl_max_slider, targ_slider, V_set_slider, VAC_min_slider, VAC_max_slider, V_ref_slider};
    for(auto& a:sliders)
    {
        a->setTickInterval(2);
        a->setRange(0,126);
        //        a->setMaximumWidth(labels_width*3);
        a->setTickPosition(QSlider::TickPosition::TicksAbove);
        connect(a,SIGNAL(sliderReleased()),this,SLOT(oneSend()));
        //        connect(a,SIGNAL(sliderReleased()),this,SLOT(oneSend()));
    }
    V_ref_slider->setRange(0,30);
    V_set_slider->setValue(20);
    targ_slider->setRange(0,60);
    VAC_mini_slider->setRange(0,30);


    serial_le=new QLineEdit("COM3");

    V_ref_slider->setValue(4);

    t2_le=new QLineEdit("6");
    dT_le=new QLineEdit("10");
    T_le=new QLineEdit("8");
    VAC_max_slider->setValue(30);
    VAC_min_slider->setValue(30);
    //    chan_le=new QLineEdit("0");
    chan_cb=new QComboBox();
    chan_cb->addItem("1",0);
    chan_cb->addItem("2",1);
    chan_cb->addItem("3",2);
    chan_cb->addItem("4",3);
    chan_cb->addItem("5",4);
    chan_cb->addItem("6",5);
    chan_cb->addItem("7",6);
    chan_cb->addItem("8",7);
    reverse_check = new QCheckBox("");

    //    serial_le->setMaximumWidth(200);
    //    //    V1_le->setMaximumWidth(200);
    //    //    V2_le->setMaximumWidth(200);
    //    V_reset_slider->setMaximumWidth(200);
    //    t2_le->setMaximumWidth(200);
    //    dT_le->setMaximumWidth(200);
    //    T_le->setMaximumWidth(200);
    //    //    VAC_min_slider->setMaximumWidth(200);
    //    //    VAC_max_slider->setMaximumWidth(200);
    //    reverse_le->setMaximumWidth(200);
    //    chan_cb->setMaximumWidth(200);

    //    COMInit();
    connect(serial_le,SIGNAL(returnPressed()),this,SLOT(COMInit()));
    //    serial_le=new QLineEdit("");
    auto* central = new QWidget;
    auto* lt=new QGridLayout;



    //    lt->addWidget(picture_label,0,0,4,4);
    picture_label->setMaximumWidth(430);

    lt->addWidget(port_label,0,4);
    lt->addWidget(serial_le,0,5);

    lt->addWidget(V_set_label,1,4);
    lt->addWidget(V_set_slider,1,5);

    lt->addWidget(V_ref_label,2,4);
    lt->addWidget(V_ref_slider,2,5);

    lt->addWidget(reset_label,3,4);
    lt->addWidget(V_reset_slider,3,5);

    lt->addWidget(VAC_min_label,4,4);
    lt->addWidget(VAC_min_slider,4,5);

    lt->addWidget(reverse_label,5,4);
    lt->addWidget(reverse_check,5,5);

    lt->addWidget(V_pl_max_label, 6,4);
    lt->addWidget(V_pl_max_slider, 6,5);




    lt->addWidget(chan_label,0,6);
    lt->addWidget(chan_cb,0,7);

    lt->addWidget(dT_label,1,6);
    lt->addWidget(dT_le,1,7);

    lt->addWidget(T_label,2,6);
    lt->addWidget(T_le,2,7);

    lt->addWidget(t2_label,3,6);
    lt->addWidget(t2_le,3,7);

    lt->addWidget(VAC_max_label,4,6);
    lt->addWidget(VAC_max_slider,4,7);

    lt->addWidget(targ_label,5,6);
    lt->addWidget(targ_slider,5,7);

    lt->addWidget(VAC_check,6,6);
    lt->addWidget(VAC_mini_slider,6,7);

    //-----------------------------
    lt->addWidget(write_check, 2,0,1,2);

    lt->addWidget(shots_btn, 3,0,1,2);
    lt->addWidget(analyze_btn, 3,2,1,2);

    lt->addWidget(gather_mult_btn, 4,0,1,2);
    lt->addWidget(separ_mult_btn, 4,2,1,2);

    lt->addWidget(custom_btn,5,0,1,2);
    lt->addWidget(vac_btn,5,2,1,2);

    lt->addWidget(prog_btn,6,0,1,2);
    lt->addWidget(rest_btn,6,2,1,2);

    lt->addWidget(rest_btn,6,2,1,2);

    //    lt->addWidget(filler_btn,9,1,1,2);
    //    lt->addWidget(filler_btn1,7,2);


    lt->addWidget(cur_plot,7,0,1,4);

    if(!imitation_on)
        lt->addWidget(set_plot,7,4,1,4);
    else
        lt->addWidget(imit_label,7,4,1,4);

    //    set_plot->setMaximumWidth(500);


    central->setLayout(lt);
    setCentralWidget(central);


    //    connect()
    connect(chan_cb,SIGNAL(currentIndexChanged(int)),this,SLOT(chanPressed()));
    connect(reverse_check,SIGNAL(stateChanged(int)),this,SLOT(oneSend()));
    connect(VAC_check,SIGNAL(stateChanged(int)),this,SLOT(VAC_check_changed()));
    //    connect(V_reset_slider,SIGNAL(sliderReleased()),this,SLOT(oneSend()));
    connect(t2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(dT_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(T_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(VAC_mini_slider,SIGNAL(sliderReleased()),this,SLOT(oneSend()));
    connect(targ_slider,SIGNAL(sliderReleased()),this,SLOT(oneSend()));


    serial_get_timer.setInterval(5);

    imit_timer.setInterval(300);
    if(imitation_on)
        imit_timer.start();
    //prog_btn->set
//    one_shot_timer.setInterval(3);

    connect(&one_shot_timer, SIGNAL(timeout()),this,SLOT(shots_btn_pressed()));
    connect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(Serial_get()));
    connect(&imit_timer,SIGNAL(timeout()),this,SLOT(setNewImg()));
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

    MD=CUSTOM;
    oneSend();

    serial_le->setDisabled(true);

    serial_get_timer.start();
}

void MainWindow::Serial_get()
{
    static auto mySign=[](int a, bool b){if(b)return a; else return -a;};
    static QString adch;

    static uint8_t buf1;
    uint16_t h;
    int a,b;
    int N=port.read(buf,buf_N);
    for(int i=0;i<N;i++)
    {
        //        qDebug()<<(uint8_t)buf[i];
        if((MD==CUSTOM))
        {
            switch(ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                    ptr=2;
                break;
            case 1:
                buf1=buf[i];
                break;
            case 2:

                ind_c=(ind_c+1)%data_adc.size();
                data_adc[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                curveADC->signalDrawing(I_koef);
                break;

            }
            ptr++;
            ptr%=3;

        }
        else if(MD==VAC)//VAC
        {
            static int8_t buf_;
            //             qDebug()<<(uint8_t)buf[i];
            //            set_plot->setAxisAutoScale(2);
            //            set_plot->setAxisAutoScale(1);
            //            set_plot->setAxisAutoScale()
            switch(ptr)
            {

            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    //                    qDebug()<<"hello";
                    ptr=4;

                }
                //                qDebug()<<"\nc:";
                break;
                //            case 1:
                //                h=((uint16_t)(uint8_t)buf[i]);

                //                break;
            case 1:

                //                qDebug()<<buf1;
                buf1=(uint8_t)buf[i];
                //                current[current_ind]=(((((uint16_t)buf[i])<<8)|buf1));
                //                current[current_ind]=256-(.01+(uint8_t)buf[i])-128;
                //                current_ind++;
                //                current_ind%=current.size();


                break;
            case 2:
                //                //                qDebug()<<(uint16_t)buf[i];
                current[current_ind]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //                current[current_ind]=(((((uint8_t)buf1))|0  ));
                current_ind++;
                current_ind%=current.size();
                break;

                //            DUMMY
                //            case 2:
                //                break;
            case 3:
                //                data_adc[ind_c]=(.01+(uint8_t)buf[i]);
                ind_c=(ind_c+1)%data_adc.size();
                //                curveADC->signalDrawing(1);
                voltage[voltage_ind]=buf_;


                if(mapIV.find((int)current[voltage_ind])!=mapIV.end())
                    voltage[voltage_ind]-=mySign(mapIV[(int)current[voltage_ind]],!reversed[chan]);

                //                                if(current[voltage_ind]<0)
                //                                    voltage[voltage_ind]-=
                //                                            mySign(
                //                                            -0.522422*current[voltage_ind]
                //                                            -0.096021*pow(current[voltage_ind],2)
                //                                            -0.003557*pow(current[voltage_ind],3)
                //                                            -0.000041*pow(current[voltage_ind],4)
                //                                                ,!reversed[chan] );


                buf_=buf[i];
                voltage_ind++;
                voltage_ind%=voltage.size();

                setCurve->set_Drawing(voltage,current,voltage_ind,current_ind);
                break;
            case 4:
                //DUMMY
                break;

            }
            ptr++;
            ptr%=5;


        }
        else if(MD==PROGRAM)
        {
            switch(ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                {

                    ptr=3;
                    ptr%=4;
                }
                break;
            case 1:


                PROGRAM_done=(uint8_t)buf[i];
                if(PROGRAM_done)
                {
                    prog_btn->setText("PROGRAM MODE: DONE");
                    //                    prog_btn->set
                }
                else
                    prog_btn->setText("PROGRAM MODE: ...");
                break;
            case 2:

                buf1=buf[i];

                break;
            case 3:
                ind_c=(ind_c+1)%data_adc.size();

                data_adc[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));

                curveADC->signalDrawing(I_koef);
                break;

            }
            ptr++;
            ptr%=4;
        }
        else if(MD==ONE_SHOT)
        {
            switch(ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    ptr=4;                    
                }
                break;
            case 1:

                buf1=(uint8_t)buf[i];
                break;
            case 2:
                //                //                qDebug()<<(uint16_t)buf[i];
                ind_c=(ind_c+1)%data_adc1.size();
                data_adc1[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                //                curveADC->signalDrawing(I_koef);

                adch=QString::number(data_adc1[ind_c]*I_koef);
                break;
            case 3:
                buf1=(uint8_t)buf[i];

                break;
            case 4:



                data_adc2[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                curveADC->twoSignalsDrawing(I_koef,data_adc1,data_adc2,curveADC2);
                if(write_check->isChecked())
                {
                    QTextStream outStream(file_rand_stat);
                    outStream << adch << "    "<<QString::number(data_adc2[ind_c]*I_koef)<<endl ;
                }
                break;

            }
            ptr++;
            ptr%=5;
        }
        else if(MD == ANALYZE)
        {
            static uint8_t h , hh;
            switch(ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    ptr=4;
                    ptr%=5;
                }
                break;

            case 1:
                h=(uint8_t)buf[i];
                break;

            case 2:
                hh=(uint8_t)buf[i];
                qDebug()<<h<<" "<<hh;
                break;

            case 3:

                buf1=(uint8_t)buf[i];



                break;
            case 4:
                //                //                qDebug()<<(uint16_t)buf[i];
                ind_c=(ind_c+1)%data_adc.size();
                data_adc[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                curveADC->signalDrawing(I_koef);

                adch=QString::number(data_adc[ind_c]*I_koef);

                QTextStream outStream(file_analyze);
                if(write_check->isChecked())
                    outStream << h << "   "<<hh<<"   "<<QString::number(data_adc[ind_c]*I_koef)<<endl ;
                break;

            }
            ptr++;
            ptr%=5;

        }
    }
}

void MainWindow::custom_btn_pressed()
{
    //    for (auto& it:data_adc)
    //        it=0;
    disconnect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(Serial_get()));
    V_set_label->setText("V set");
    //    reset_label->setText("tau1");
    t2_label->setText("tau2");
    MD=CUSTOM;
    oneSend();
    connect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(Serial_get()));
}

void MainWindow::prog_btn_pressed()
{
    MD=PROGRAM;
    //    reset_label->setText("targ");
    //    t2_label->setText("V+ max");
    oneSend();
    data_adc.resize(data_adc.size(),0);
    for (auto& it:data_adc)
        it=0;
}
void MainWindow::rest_btn_pressed()
{
    MD=CUSTOM;

    char c;
    c=255;
    port.write(&c,1);
    //    case 0:

    c=MD;
    port.write(&c,1);

    c=0;
    port.write(&c,1);

    port.write(&c,1);

    //    c=V_reset_slider->value();
    c=0;
    port.write(&c,1);




    c=t2_le->text().toInt();
    port.write(&c,1);

    c=dT_le->text().toInt();
    port.write(&c,1);

    c=T_le->text().toInt();
    port.write(&c,1);

    c=chan;
    port.write(&c,1);

    c=reverse_check->isChecked();
    port.write(&c,1);
}

void MainWindow::gather_mult_btn_pressed()
{
    MD=GATHER_MULT;
    oneSend();
}

void MainWindow::separ_mult_btn_pressed()
{
    ptr=0;
    MD=SEPAR_MULT;
    oneSend();
}

void MainWindow::shots_btn_pressed()
{
    ptr=0;
    MD = ONE_SHOT;
    oneSend();
}

void MainWindow::analyze_btn_pressed()
{
    MD = ANALYZE;
    file_analyze -> open(QIODevice::WriteOnly);
    oneSend();
}

void MainWindow::write_check_state_changed(int x)
{
    if(x==0)
    {
        file_analyze->close();
        file_rand_stat->close();
    }
    else
    {
        file_rand_stat->open(QIODevice::WriteOnly);
    }
}

void MainWindow::chanPressed()
{
    rest_btn_pressed();
    QThread::msleep(100);


    reversed[chan]=reverse_check->isChecked();
    VAC_min[chan]=VAC_min_slider->value();
    VAC_max[chan]=VAC_max_slider->value();

    chan = chan_cb->currentIndex();
    qDebug()<<chan;

    VAC_min_slider->setValue((VAC_min[chan]));
    //    VAC_max_le->setText(QString::number(VAC_max[chan]));
    VAC_max_slider->setValue((VAC_max[chan]));
    //    reverse_le->setText(QString::number(reversed[chan]));
    oneSend();

}

void MainWindow::VAC_check_changed()
{
    MD=VAC;

    oneSend();
}

void MainWindow::vac_btn_pressed()
{
    ptr=0;
    set_plot->setAxisAutoScale(QwtPlot::xBottom,1);
    set_plot->setAxisAutoScale(QwtPlot::xTop,1);
    set_plot->setAxisAutoScale(QwtPlot::yLeft,1);
    set_plot->setAxisAutoScale(QwtPlot::yRight,1);
    disconnect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(Serial_get()));
    //    reset_label->setText("tau1");
    t2_label->setText("tau2");
    for(auto& it:current)
        it=0;
    current_ind=0;

    for(auto& it:voltage)
        it=0;
    voltage_ind=0;


    MD=VAC;

    V_set_label->setText("V set");
    connect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(Serial_get()));


    oneSend();
    data_adc.resize(data_adc.size(),0);
    for (auto& it:data_adc)
        it=0;

    curveADC->signalDrawing(1);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    //    QString filename = "Data.txt";
    //    mapIV
    static bool key1=0;
    switch(event->key())
    {

    // bad place in a code
    case Qt::Key_1:
//        key1=!key1;
//        if(key1)
//            one_shot_timer.start(43);
//        else
//            one_shot_timer.stop();
        break;

    case Qt::Key_W:

        WriteFile("ShortCircuit");

        break;

    case Qt::Key_R:
        ReadFile("ShortCircuit",mapIV);
        break;


    case Qt::Key_E:
        mapIV.erase(mapIV.begin(),mapIV.end());
        break;

    case Qt::Key_P:
        static int cnt=1;
        QPixmap pix = QPixmap::grabWidget(set_plot);
        pix.save(QString("E:/pics/graph")+QString::number(cnt)+QString(".png"),"PNG");
        cnt++;
    }

}

void MainWindow::setNewImg()
{
    static int cnt=0;
    QImage image(QString("E:\\pics\\")+QString("graph")+QString::number(cnt)+QString(".png"));
    imit_label->setPixmap(QPixmap::fromImage(image));
    //    qDebug()<<"hey";
    cnt++; if(cnt>260)cnt=1;
}

void MainWindow::oneSend()
{
    serial_get_timer.setInterval(5);
    char c;
    V_pl_max_label->setText("V+max: "+QString().setNum(V_koef*V_pl_max_slider->value(), 'g',2));
    targ_label->setText("targ: "+QString().setNum(I_koef*targ_slider->value(), 'g',3));
    VAC_min_label->setText("VAC- "+QString().setNum(V_koef*VAC_min_slider->value(), 'g',2));
    VAC_max_label->setText("VAC+ "+QString().setNum(V_koef*VAC_max_slider->value(), 'g',2));
    V_set_label->setText("V set: "+QString().setNum(V_koef*V_set_slider->value(), 'g',2));
    reset_label->setText("V reset: "+QString().setNum(V_koef*V_reset_slider->value(), 'g',2));
    V_ref_label->setText("Ref: "+QString().setNum(V_koef*V_ref_slider->value(), 'g',2));
    VAC_check->setText("check: "+QString().setNum(V_koef*VAC_mini_slider->value(), 'g',2));

    c=255;
    port.write(&c,1);
    //    case 0:

    c=MD;
    port.write(&c,1);//1


    if((MD==VAC))
        if(VAC_check->isChecked())
            c=0;
        else
            if(!reverse_check->isChecked())
                c=VAC_min_slider->value();
            else
                c=VAC_max_slider->value();
    else if(MD==CUSTOM)
        c=V_set_slider->value();
    else if(MD==PROGRAM)
        c=V_set_slider->value();
    else if((MD == ANALYZE) || (MD == ONE_SHOT))
    {
        c=-V_set_slider->value();
    }
    //2
    port.write(&c,1);


    if(MD==VAC)
        if(VAC_check->isChecked())
            c=VAC_mini_slider->value();
        else
            if(!reverse_check->isChecked())
                c=VAC_max_slider->value();
            else
                c=VAC_min_slider->value();

    else
        c=V_ref_slider->value();
    port.write(&c,1);//3

    if(MD!=PROGRAM)
        c=V_reset_slider->value();
    else
        c=targ_slider->value();//target
    port.write(&c,1);//4

    if(MD!=PROGRAM)
        c=t2_le->text().toInt();
    else
        c=V_pl_max_slider->value();
    port.write(&c,1);//5

    c=dT_le->text().toInt();
    port.write(&c,1);//6

    c=T_le->text().toInt();
    port.write(&c,1);//7

    c=chan;
    port.write(&c,1);//8

    c=reverse_check->isChecked();
    reversed[chan]=reverse_check->isChecked();
    port.write(&c,1);//9


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
    //    char c='s';
    //    while(1)
    //        port.write(&c,1);
    //    qDebug()<<"destructor is here";
    //    MD=CUSTOM;
    //    oneSend();
}



void WriteFile(QString s)
{
    QFile file(s);
    if (file.open(QIODevice::WriteOnly)) {
        QTextStream stream(&file);
        for(int i =0;i<voltage.size();i++)
            //            if(current[i]<0)
            stream << voltage[i] <<":"<<current[i]<<"  ";
    }
    file.close();
}


void ReadFile(QString s, map<int,int>& m)
{
    QFile inputFile(s);
    if (inputFile.open(QIODevice::ReadOnly))
    {
        QTextStream in(&inputFile);
        while (!in.atEnd())
        {
            QString line = in.readLine();

            QStringList list = line.split(" ",QString::SkipEmptyParts);
            for(auto& a:list)
            {

                auto b = a.split(":");
                auto it=m.find(b[1].toInt());
                if(it==m.end())
                    m.emplace(b[1].toInt(),b[0].toInt());
            }
            for(auto& a :m)
            {
                //                qDebug()<<a.first<<":"<<a.second;
            }
            map<int,int> m_;
            for(auto it=m.begin();it!=next(m.end(),-1);it++)
            {
                for(auto i = it->first; i!= next(it,1)->first;i++)
                    m_.emplace(i,
                               it->second+(i-it->first)*(float)(next(it,1)->second-it->second)/(next(it,1)->first-it->first)
                               );
                //                qDebug()<<it->first;
            }
            m=m_;
        }
        inputFile.close();
    }
}
