#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void oneSend();
    void COMInit();
    void readADC();
    void vac_btn_pressed();
};

#endif // MAINWINDOW_H
