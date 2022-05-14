#ifndef DRAWING_H
#define DRAWING_H
#include <qwt_plot.h>
#include <qwt_plot_grid.h>
#include <qwt_legend.h>
#include <qwt_plot_curve.h>
#include <qwt_symbol.h>
#include <qwt_plot_magnifier.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_picker.h>
#include <qwt_picker_machine.h>
//#include"serial.h"
using namespace std;


class myCurve:public QwtPlotCurve
{
public:


    int& ind_c;
    vector<float> &data;
    QwtPlot* d_plot;
    QwtSymbol *symbol;


    myCurve(int bufShowSize, vector<float> &dataH,QwtPlot* d_plotH,const QString &title,
            const QColor &color, const QColor &colorSymbol,int& ind_ch );
    myCurve(QwtPlot *d_plotH, QString title, QColor color);
    void twoSignalsDrawing(float k, vector<float>& data1, vector<float>& data2, QwtPlotCurve*);
    void signalDrawing(float k);
    void pointDrawing(float , float);
    void set_Drawing(vector<float> &x, vector<float> &y, int i1=-1, int i2=-1);

    void fourSignalsDrawing(float k, vector<float>& data1, vector<float>& data2,
                                     vector<float>& data3, vector<float>& data4, vector<QwtPlotCurve*>);

};



#endif // DRAWING_H
