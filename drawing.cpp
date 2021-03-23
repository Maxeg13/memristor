#include "drawing.h"
#include "drawing.h"
#include "drawing.h"
extern float V_koef;
extern float I_koef;
vector<float> utilvec;
int utilint;
const int pen_size = 2;



myCurve::myCurve(int bufShowSize, vector<float> &dataH,QwtPlot* d_plotH,const QString &title,
                 const QColor &color, const QColor &colorSymbol,int& ind_ch ):
   QwtPlotCurve(), data(dataH),ind_c(ind_ch)
{
    d_plot=d_plotH;
    setTitle(title);
    setPen(color,2);


    dataH.resize(bufShowSize);
    for(int i=0;i<dataH.size();i++)
    {
        dataH[i ]=0;
    }
}


myCurve::myCurve(QwtPlot* d_plotH, QString title,
                 QColor color):
    QwtPlotCurve(),data(utilvec),ind_c(utilint)
{
    d_plot=d_plotH;
    setTitle(title);
    setPen(color,pen_size);
}


void myCurve::signalDrawing(float k)
{
    // Добавить точки на ранее созданную кривую
    QPolygonF points;

    for (int i=1;i<data.size();i++)
    {
        points<<QPointF(i,data[(ind_c+i+1)%data.size()]*k);
    }
    setSamples( points ); // ассоциировать набор точек с кривой
    attach( d_plot); // отобразить кривую на графике
}

void myCurve::twoSignalsDrawing(float k, vector<float>& data1, vector<float>& data2, QwtPlotCurve* curve2)
{
//    vector<float> &data;
    // Добавить точки на ранее созданную кривую
    QPolygonF points;

    for (int i=1;i<data.size();i++)
    {
        points<<QPointF(i,data1[(ind_c+i+1)%data1.size()]*k);
    }
    setSamples( points ); // ассоциировать набор точек с кривой
    attach( d_plot); // отобразить кривую на графике

    QPolygonF points2;
    curve2->setPen(Qt::red, pen_size);

    for (int i=1;i<data2.size();i++)
    {
        points2<<QPointF(i,data2[(ind_c+i+1)%data2.size()]*k);
    }
    curve2->setSamples( points2 ); // ассоциировать набор точек с кривой
    curve2->attach( d_plot); // отобразить кривую на графике
}

void myCurve::pointDrawing(float x,float y)
{
    // Добавить точки на ранее созданную кривую
    QPolygonF points;


    points<<QPointF(x,y);

    setSamples( points ); // ассоциировать набор точек с кривой
    attach( d_plot); // отобразить кривую на графике
}

void myCurve::set_Drawing(vector<float>& x, vector<float>& y, int i1, int i2 )
{
    // Добавить точки на ранее созданную кривую
    QPolygonF points;

    int s=x.size();
    for(int i=0;i<(s);i++)
        if((i!=i1)&&(i!=i2))
            points<<QPointF(V_koef*x[(i+i1+s)%s], I_koef*y[(i+i1+s)%s]);

    setSamples( points ); // ассоциировать набор точек с кривой
    attach( d_plot); // отобразить кривую на графике
}
