//v1 , v2 are positive for VAC_mode (convert to -v1 and v2)
//20 us to 3.5 ms , 1V to 5V

// shortcircuit  commented out

//1         9
//17        25
//33        41
//49        57

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

bool imitation_on =
        false;
//                true;

//0.070
//0.06
float V_koef=0.1;

//11.
float I_koef=5.7;
using namespace std;
QPushButton *vac_btn,  *prog_btn, *filler_btn, *reset_btn,
*filler_btn1, *rest_btn, *gather_mult_btn, *separ_mult_btn, *shots_btn;
QPushButton* parse_input_btn;
QPushButton *theta_btn;
//QPushButton *measure_btn;

#define CHAN_N 64

enum MODE
{
    CUSTOM,
    VAC,
    PROGRAM,
    GATHER_MULT,
    SEPAR_MULT,
    ONE_SHOT,
    ANALYZE,
    THETA,
    RESET,
    JSON_INPUTS,
    JSON_MEASURE,
    JSON_MEASURE_CLOSE,
    JSON_PROGRAM
};

MODE MD;
int json_chan;
int json_targ;
vector<int> chans_to_meas;
map<int, int> json_vs;
map<int, int> json_ws;
bool abstractIs;
map<int,int> mapIV;
int reversed[CHAN_N];
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
QwtPlotCurve  *curveADC3;
QwtPlotCurve  *curveADC4;
//QwtPlot* ;
int ind_c;
int ind_c1;
int ind_c2;
QLabel* V_set_label;
QLabel* V_pl_max_label;
QLabel* theta_label;
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
QLabel* infoLabel;
QLabel* VAC_max_label;
QSlider* targ_slider;
QSlider* theta_v_slider;
QSlider* VAC_min_slider;
int VAC_min[CHAN_N];
int VAC_max[CHAN_N];
int theta[CHAN_N];
QSlider* VAC_max_slider;
QComboBox* chan_cb;
QCheckBox* reverse_check;
QCheckBox* abstract_check;
QLabel* reset_label;
QLabel* t2_label;

//QLabel* t2_label;
uint8_t PROGRAM_done=0;
float V1;

float mvs[]={296, 480, 1020, 1960, 3000};
float grid[]={10, 20 ,80, 100, 163};

uint8_t receive_ptr=0;
QLabel *imit_label;
vector<float> data_adc;
vector<float> data_adc1;
vector<float> data_adc2;
vector<float> data_adc3;
vector<float> data_adc4;
QLabel* V_ref_label;
vector<float> current;
int current_ind;
vector<float> voltage;
int voltage_ind;
const int buf_N=30;
char buf[buf_N];

QTimer json_program_timer;
QTimer measure_timer;
QTimer json_input_send_timer;
QTimer VAC_send_timer;
QTimer serial_get_timer;
QTimer imit_timer;
QTimer one_shot_timer;
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
    int V_m_ = 25;
    for(int i=0;i<CHAN_N; i++)
    {
    reversed[i] = 0;
    VAC_max[i] = VAC_min[i] = V_m_;
    }


    mapIV.erase(mapIV.begin(),mapIV.end());
    curveADC2=new QwtPlotCurve;
    curveADC3=new QwtPlotCurve;
    curveADC4=new QwtPlotCurve;
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

    json_program_timer.setInterval(40);
    json_input_send_timer.setInterval(40);
    measure_timer.setInterval(40);
    connect(&json_program_timer, SIGNAL(timeout()), this, SLOT(json_program_timeout()));
    connect(&json_input_send_timer, SIGNAL(timeout()),this,SLOT(jsonInputs()));
    connect(&measure_timer, SIGNAL(timeout()), this, SLOT(json_measure_timeout()));
    connect(&VAC_send_timer, SIGNAL(timeout()), this, SLOT(vac_send_timeout()));

    // buttons

    rest_btn= new QPushButton("set zero voltage");
    prog_btn=new QPushButton("PROGRAM MODE");
    theta_btn=new QPushButton("THETA MODE");
    gather_mult_btn = new QPushButton("gather mult");
    separ_mult_btn = new QPushButton("usual mult");
    shots_btn = new QPushButton("COMMON SET");
    reset_btn = new QPushButton("RESET");
    vac_btn=new QPushButton("VAC MODE");
    VAC_check=new QCheckBox("check: ");
    write_check = new QCheckBox("write on");
    parse_input_btn = new QPushButton("set json input");

    //////
    /// brief connect
    ///

    connect(parse_input_btn, SIGNAL(pressed()),this,SLOT(setInputJson()));
    connect(write_check, SIGNAL(stateChanged(int)), this, SLOT(write_check_state_changed(int)));
    connect(shots_btn,SIGNAL(pressed()),this,SLOT(shots_btn_pressed()));
    connect(separ_mult_btn,SIGNAL(pressed()),this,SLOT(separ_mult_btn_pressed()));
    connect(gather_mult_btn,SIGNAL(pressed()),this,SLOT(gather_mult_btn_pressed()));
    connect(rest_btn,SIGNAL(pressed()),this,SLOT(rest_btn_pressed()));
    connect(vac_btn,SIGNAL(pressed()),this,SLOT(vac_btn_pressed()));
    connect(reset_btn,SIGNAL(pressed()),this,SLOT(reset_btn_pressed()));
    connect(theta_btn,SIGNAL(pressed()),this,SLOT(theta_btn_pressed()));
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
    data_adc3.resize(bufShowSize);
    data_adc4.resize(bufShowSize);
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

    theta_label = new QLabel("theta: ");

    QLabel* dT_label=new QLabel("dT");
    dT_label->setMaximumWidth(labels_width);

    QLabel* T_label=new QLabel("T");
    T_label->setMaximumWidth(labels_width);

    VAC_min_label=new QLabel("VAC-");
    VAC_min_label->setMaximumWidth(labels_width);

    VAC_max_label=new QLabel("VAC+");
    VAC_max_label->setMaximumWidth(labels_width);

    infoLabel = new QLabel("channels order: black, blue, green, red");

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
    theta_v_slider=new QSlider(Qt::Orientation::Horizontal);
    vector<QSlider*> sliders={V_reset_slider, VAC_mini_slider, V_pl_max_slider,
                              targ_slider, V_set_slider, VAC_min_slider, VAC_max_slider,
                              V_ref_slider, theta_v_slider};
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
    theta_v_slider->setRange(-126, 126);


    serial_le=new QLineEdit("COM3");

    V_ref_slider->setValue(4);

    t2_le=new QLineEdit("6");
    dT_le=new QLineEdit("10");
    T_le=new QLineEdit("8");
    VAC_max_slider->setValue(30);
    VAC_min_slider->setValue(30);
    //    chan_le=new QLineEdit("0");
    chan_cb=new QComboBox();

    for (int i =0 ;i<CHAN_N; i++)
    {
        auto itemText = QString::number(i+1)+ " / "+ QString::number((i%8+1)) + " " + (((i/8)%2==1)?QString("down"):QString("up"));
        chan_cb->addItem(itemText,i);
    }

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

    lt->addWidget(theta_label,7,4);
    lt->addWidget(theta_v_slider,7,5,1,3);



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
    lt->addWidget(infoLabel, 2, 2);

    lt->addWidget(shots_btn, 3,0,1,2);
    lt->addWidget(reset_btn, 3,2,1,2);

    lt->addWidget(gather_mult_btn, 4,0,1,2);
    lt->addWidget(separ_mult_btn, 4,2,1,2);

    lt->addWidget(theta_btn,5,0,1,2);
    lt->addWidget(vac_btn,5,2,1,2);

    lt->addWidget(prog_btn,6,0,1,2);
    lt->addWidget(rest_btn,6,2,1,2);

    lt->addWidget(rest_btn,6,2,1,2);
    lt->addWidget(parse_input_btn, 7,0,1,2);

    //    lt->addWidget(filler_btn,9,1,1,2);
    //    lt->addWidget(filler_btn1,7,2);


    lt->addWidget(cur_plot,8,0,1,4);

    if(!imitation_on)
        lt->addWidget(set_plot,8,4,1,4);
    else
        lt->addWidget(imit_label,8,4,1,4);

    //    set_plot->setMaximumWidth(500);


    central->setLayout(lt);
    setCentralWidget(central);

    connect(chan_cb,SIGNAL(currentIndexChanged(int)),this,SLOT(chanPressed()));
    connect(reverse_check,SIGNAL(stateChanged(int)),this,SLOT(oneSend()));
    connect(VAC_check,SIGNAL(stateChanged(int)),this,SLOT(VAC_check_changed()));
    //    connect(V_reset_slider,SIGNAL(sliderReleased()),this,SLOT(oneSend()));
    connect(t2_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(dT_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(T_le,SIGNAL(returnPressed()),this,SLOT(oneSend()));
    connect(VAC_mini_slider,SIGNAL(sliderReleased()),this,SLOT(oneSend()));
    connect(targ_slider,SIGNAL(sliderReleased()),this,SLOT(oneSend()));
    connect(theta_v_slider, SIGNAL(sliderReleased()),this,SLOT(oneSend()));

    serial_get_timer.setInterval(5);
    VAC_send_timer.setInterval(18);

    imit_timer.setInterval(300);
    if(imitation_on)
        imit_timer.start();
    //prog_btn->set
//    one_shot_timer.setInterval(3);

//    connect(&one_shot_timer, SIGNAL(timeout()),this,SLOT(shots_btn_pressed()));
    connect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(oneGet()));
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

void MainWindow::oneGet()
{
    static auto mySign=[](int a, bool b){if(b)return a; else return -a;};
    static QString adch[4];

    static uint8_t buf1;
    uint16_t h;
    int16_t hi;
    int a,b;
    int N=port.read(buf,buf_N);

    for(int i=0;i<N;i++)
    {
        //        qDebug()<<(uint8_t)buf[i];
        if((MD==CUSTOM))
        {
            switch(receive_ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                    receive_ptr=4;
                break;
            case 3:
                buf1=buf[i];
                break;
            case 4:

                ind_c=(ind_c+1)%data_adc.size();
                data_adc[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                curveADC->signalDrawing(I_koef);
                break;
            }

            receive_ptr++;
            receive_ptr%=5;

        }else if((MD==JSON_MEASURE_CLOSE)) {  // cheat indeed
            switch(receive_ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                    receive_ptr=4;
                break;
            case 3:
                buf1=buf[i];
                break;
            case 4:

                ind_c=(ind_c+1)%data_adc.size();
                hi = 512-(((((uint8_t)buf[i]<<8))|buf1  ));

                qDebug()<<"channel: "<<json_chan<<", w: " << hi;
                break;
            }

            receive_ptr++;
            receive_ptr%=5;
        }
        else if(MD==VAC)//VAC
        {
            static int8_t buf_;
            //             qDebug()<<(uint8_t)buf[i];
            //            set_plot->setAxisAutoScale(2);
            //            set_plot->setAxisAutoScale(1);
            //            set_plot->setAxisAutoScale()
            switch(receive_ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    receive_ptr=4;
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

            }
            receive_ptr++;
            receive_ptr%=5;


        }
        else if((MD==PROGRAM)||(MD==JSON_PROGRAM))
        {
            static uint8_t  _PROGRAM_done=0;
            switch(receive_ptr)
            {
            case 0:

                if((uint8_t)buf[i]!=255)
                {
                    receive_ptr=4;
                }
                break;
            case 1:
                break;
            case 2:
                buf1=buf[i];

                break;
            case 3:
                ind_c=(ind_c+1)%data_adc.size();

                data_adc[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                if(!_PROGRAM_done && PROGRAM_done)
                    qDebug()<<data_adc[ind_c]*I_koef;
//                qDebug()<<data_adc[ind_c];
                curveADC->signalDrawing(I_koef);
                break;
            case 4:
                _PROGRAM_done = PROGRAM_done;
                PROGRAM_done=(uint8_t)buf[i];
                if(PROGRAM_done)
                {
                    prog_btn->setText("PROGRAM MODE: DONE");
                    //                    prog_btn->set
                }
                else
                    prog_btn->setText("PROGRAM MODE: ...");
                break;
            }
            receive_ptr++;
            receive_ptr%=5;
        }
        else if((MD==THETA)||(MD==JSON_INPUTS))
        {
            switch(receive_ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    receive_ptr=4;
                }
                break;
            case 1:

                break;
            case 2:

                buf1=buf[i];

                break;
            case 3:
                ind_c=(ind_c+1)%data_adc.size();

                data_adc[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));

                curveADC->signalDrawing(I_koef);
                break;
            case 4:
                break;

            }
            receive_ptr++;
            receive_ptr%=5;
        }
        else if((MD==ONE_SHOT)||(MD==RESET))
        {
            switch(receive_ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    receive_ptr=4;
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

                adch[0]=QString::number(data_adc1[ind_c]*I_koef);
                break;
            case 3:
                buf1=(uint8_t)buf[i];

                break;
            case 4:
                data_adc2[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                adch[1]=QString::number(data_adc2[ind_c]*I_koef);
                break;
            case 5:
                buf1=(uint8_t)buf[i];

                break;
            case 6:
                data_adc3[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                adch[2]=QString::number(data_adc3[ind_c]*I_koef);
                break;
            case 7:
                buf1=(uint8_t)buf[i];

                break;
            case 8:
                data_adc4[ind_c]=512-(((((uint8_t)buf[i]<<8))|buf1  ));
                //            data_adc[ind_c]=18000./(0.01+(100./(V1))*adc2mvs((uint8_t)buf[i]));
                vector<QwtPlotCurve*> curves{curveADC2, curveADC3, curveADC4};
                curveADC->fourSignalsDrawing(I_koef,data_adc1,data_adc2,
                                             data_adc3,data_adc4,curves);

                map<int, QString> mode_names{{RESET, "RESET MODE:"}, {ONE_SHOT, "SET MODE:  "}};
                QString str = mode_names[(int)MD] + "    " + adch[0] + "    " + adch[1] + "    " + adch[2] + "    " + QString::number(data_adc4[ind_c]*I_koef);
                qDebug()<< str<<endl;
                if(write_check->isChecked())
                {
                    QTextStream outStream(file_rand_stat);                    
                    outStream << str<<endl;
                }
                break;
            }
            receive_ptr++;
            receive_ptr%=(5+5);
        }
        else if(MD == ANALYZE)
        {
            static uint8_t h , hh;
            switch(receive_ptr)
            {
            case 0:
                if((uint8_t)buf[i]!=255)
                {
                    receive_ptr=4;
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

                adch[0]=QString::number(data_adc[ind_c]*I_koef);

                QTextStream outStream(file_analyze);
                if(write_check->isChecked())
                    outStream << h << "   "<<hh<<"   "<<QString::number(data_adc[ind_c]*I_koef)<<endl ;
                break;

            }
            receive_ptr++;
            receive_ptr%=5;

        }
    }


}

void MainWindow::theta_btn_pressed()
{
    mandatoryPrep();
    MD=THETA;
    oneSend();
    data_adc.resize(data_adc.size(),0);
    for (auto& it:data_adc)
        it=0;
    connect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(oneGet()));
}

void MainWindow::prog_btn_pressed()
{
    mandatoryPrep();
    PROGRAM_done = 0;
    MD=PROGRAM;
    //    reset_label->setText("targ");
    //    t2_label->setText("V+ max");
    data_adc.resize(data_adc.size(),0);
    for (auto& it:data_adc)
        it=0;

    oneSend();
}
void MainWindow::rest_btn_pressed()
{

    mandatoryPrep();
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
    mandatoryPrep();
    MD=GATHER_MULT;
    oneSend();
}

void MainWindow::separ_mult_btn_pressed()
{
    mandatoryPrep();
    MD=SEPAR_MULT;
    oneSend();
}

void MainWindow::shots_btn_pressed()
{
    mandatoryPrep();
    MD = ONE_SHOT;
    oneSend();

    char buf[10];
    port.read(buf,buf_N);
}

void MainWindow::reset_btn_pressed()
{   
    mandatoryPrep();
    MD = RESET;
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
    mandatoryPrep();
    MD = THETA;
    QThread::msleep(100);

    theta[chan]=theta_v_slider->value();
    reversed[chan]=reverse_check->isChecked();
    VAC_min[chan]=VAC_min_slider->value();
    VAC_max[chan]=VAC_max_slider->value();

    chan = chan_cb->currentIndex();
    qDebug()<<chan;

    theta_v_slider->setValue(theta[chan]);
    VAC_min_slider->setValue((VAC_min[chan]));
    //    VAC_max_le->setText(QString::number(VAC_max[chan]));
    VAC_max_slider->setValue((VAC_max[chan]));
    //    reverse_le->setText(QString::number(reversed[chan]));
    oneSend();

}

void MainWindow::VAC_check_changed()
{
    mandatoryPrep();

    VAC_send_timer.start();
    MD=VAC;

    oneSend();
}

void MainWindow::vac_btn_pressed()
{
    mandatoryPrep();

    set_plot->setAxisAutoScale(QwtPlot::xBottom,1);
    set_plot->setAxisAutoScale(QwtPlot::xTop,1);
    set_plot->setAxisAutoScale(QwtPlot::yLeft,1);
    set_plot->setAxisAutoScale(QwtPlot::yRight,1);
//    disconnect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(oneGet()));
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
    connect(&serial_get_timer, SIGNAL(timeout()),this,SLOT(oneGet()));

    data_adc.resize(data_adc.size(),0);
    for (auto& it:data_adc)
        it=0;

    curveADC->signalDrawing(1);
    VAC_send_timer.start();
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

//        WriteFile("ShortCircuit");

        break;

    case Qt::Key_R:
//        ReadFile("ShortCircuit",mapIV);

        mandatoryPrep();
        MD = JSON_MEASURE;
        parseJson();
        measure_timer.start();
        break;


    case Qt::Key_E:
        mapIV.erase(mapIV.begin(),mapIV.end());
        break;

    case Qt::Key_P:

        mandatoryPrep();
        parseJson();
        MD=JSON_PROGRAM;
        json_program_timer.start();


//        static int cnt=1;
//        QPixmap pix = QPixmap::grabWidget(set_plot);
//        pix.save(QString("E:/pics/graph")+QString::number(cnt)+QString(".png"),"PNG");
//        cnt++;
    }

}

void MainWindow::json_program_timeout()
{

    static int timeout_cnt = 0;


    if(json_ws.empty()) {
        qDebug()<<"json ws empty";
        json_program_timer.stop();
        return;
    }


    if(timeout_cnt < 100) {
        if(!PROGRAM_done) {
            timeout_cnt++;
            return;
        }
        else {
            qDebug()<<"channel: " << json_chan << ", program succeded";
        }
    }
    else{
        qDebug()<<"channel: " << json_chan << ", program failed";

    }


    json_chan = json_ws.begin()->first;
    json_targ = json_ws.begin()->second;
    json_ws.erase(json_ws.begin());
    timeout_cnt = 0;

    oneSend();

}

void MainWindow::json_measure_timeout()
{
    if(MD == JSON_MEASURE) {
        if(!chans_to_meas.size()) {
            measure_timer.stop();
            return;
        }

        json_chan = chans_to_meas.back();

        chans_to_meas.pop_back();
        oneSend();
        MD = JSON_MEASURE_CLOSE;
    } else if(MD == JSON_MEASURE_CLOSE) {
        oneSend();
        MD = JSON_MEASURE;
    }
}

void MainWindow::vac_send_timeout()
{
    oneSend();
}

void MainWindow::setNewImg()
{
    static int cnt=0;
    QImage image(QString("E:\\pics\\")+QString("graph")+QString::number(cnt)+QString(".png"));
    imit_label->setPixmap(QPixmap::fromImage(image));
    //    qDebug()<<"hey";
    cnt++; if(cnt>260)cnt=1;
}

void MainWindow::jsonInputs() {
    MD = JSON_INPUTS;
    oneSend();
    qDebug() << "MD is: " << static_cast<int>(MD);
    if(json_vs.empty()) {
        json_input_send_timer.stop();
    }
}

#include <QJsonObject>
void MainWindow::parseJson() {
    qDebug() << "parseJson";

    QJsonValue value;

    QFile inFile("input_data.json");
    inFile.open(QIODevice::ReadOnly|QIODevice::Text);
    QByteArray data = inFile.readAll();
    qDebug() << "data size of input file: " << data.size();
    inFile.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (doc.isNull()) {
        qDebug() << "Parse failed";
        return;
    }

    qDebug() << doc;
    QJsonObject sett2 = doc.object();

    double inp_coeff = sett2.value("input coeff").toDouble();
    qDebug() << inp_coeff;

    json_vs.clear();
    value = sett2.value(QString("inputs"));
    for(const auto& el: value.toArray()) {
        int chan = el.toArray()[0].toInt();
        json_vs[chan] = el.toArray()[1].toDouble() * inp_coeff;
    }

    chans_to_meas.clear();
    value = sett2.value(QString("channels to measure"));
    for(const auto& el: value.toArray()) {
        chans_to_meas.push_back(el.toInt());
    }
    qDebug()<<"chans to measure, size: "<<chans_to_meas.size();

    json_ws.clear();
    value = sett2.value(QString("weights targeted"));
    for(const auto& el: value.toArray()) {
        int chan = el.toArray()[0].toInt();
        json_ws[chan] = el.toArray()[1].toInt();
    }
    qDebug()<<"ws targeted, size: "<<json_ws.size();

}

void MainWindow::mandatoryPrep()
{
    json_program_timer.stop();
    VAC_send_timer.stop();
    receive_ptr = 0;
}

void MainWindow::setInputJson() {
    mandatoryPrep();
    parseJson();
    json_input_send_timer.start();
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

    theta_label->setText("theta_v: " + QString().setNum(V_koef*theta_v_slider->value(), 'g',2));

    c=255;
    port.write(&c,1);


    if(MD==JSON_INPUTS) {
        c = THETA;
    } else if((MD==JSON_MEASURE)||(MD==JSON_MEASURE_CLOSE)) {
        c = CUSTOM;
    } else if(MD==JSON_PROGRAM) {
        c = PROGRAM;
    }
    else {
        c = MD;
    }
    port.write(&c,1);//1


    if((MD==VAC))
        if(VAC_check->isChecked())
            c=0;
        else
            if(!reverse_check->isChecked())
                c=VAC_min_slider->value();
            else
                c=VAC_max_slider->value();
    else if((MD == ANALYZE) || (MD == ONE_SHOT) ||
            (MD==PROGRAM) || (MD==CUSTOM) || (MD==RESET) || (MD==JSON_MEASURE) ||
            (MD==JSON_PROGRAM))
    {
        c=VAC_mini_slider->value();
    } else if((MD==JSON_MEASURE_CLOSE)) {
        c = 0;
    }
    else if(MD == THETA) {
        c= theta_v_slider->value();
        qDebug()<<theta_v_slider->value();
    } else if(MD == JSON_INPUTS) {
        json_chan = json_vs.begin()->first;
        c = json_vs.begin()->second;
        json_vs.erase(json_vs.begin());
        qDebug()<<"json channel:"<< json_chan << ", json input: "<<(int)c * V_koef <<" V";
    }
    //2
    port.write(&c,1); // x!


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

    if(MD==PROGRAM)
        c=targ_slider->value();//target
    else if(MD==JSON_PROGRAM) {
        c = json_targ;
    }
    else
        c=V_reset_slider->value();

    port.write(&c,1);//4

    if((MD==PROGRAM)||(MD==JSON_PROGRAM))
        c=V_pl_max_slider->value();
    else
        c=t2_le->text().toInt();
    port.write(&c,1);//5

    c=dT_le->text().toInt();
    port.write(&c,1);//6

    c=T_le->text().toInt();
    port.write(&c,1);//7

    if((MD==JSON_INPUTS)||(MD==JSON_MEASURE)||(MD==JSON_MEASURE_CLOSE) ||
            (MD==JSON_PROGRAM)){
        c=json_chan;
    } else {
        c=chan;
    }

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
