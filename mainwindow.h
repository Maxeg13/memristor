#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();




public slots:
//    void restSend(int chan_);
    void keyReleaseEvent(QKeyEvent *event);
private slots:
    void setNewImg();
    void chanPressed();
    void VAC_check_changed();
    void oneSend();
    void COMInit();
    void Serial_get();
    void vac_btn_pressed();
    void prog_btn_pressed();
    void custom_btn_pressed();
    void rest_btn_pressed();
    void mult1_btn_pressed();
//    void vac_btn_pressed();
};

#endif // MAINWINDOW_H
