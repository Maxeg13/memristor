#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void parseJson();

public slots:
//    void restSend(int chan_);
    void keyReleaseEvent(QKeyEvent *event);
private slots:
    void vac_send_timeout();
    void jsonSend();
    void parseInputJson();
    void setNewImg();
    void chanPressed();
    void VAC_check_changed();
    void oneSend();
    void COMInit();
    void oneGet();
    void vac_btn_pressed();
    void prog_btn_pressed();
    void theta_btn_pressed();
    void rest_btn_pressed();
    void gather_mult_btn_pressed();
    void separ_mult_btn_pressed();
    void shots_btn_pressed();
    void reset_btn_pressed();
    void write_check_state_changed(int);
//    void vac_btn_pressed();
};

#endif // MAINWINDOW_H
