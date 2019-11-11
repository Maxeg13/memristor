#include "drawing.h"

vector<float> utilvec;
int utilint;



myCurve::myCurve(int bufShowSize, vector<float> &dataH,QwtPlot* d_plotH,const QString &title,
                 const QColor &color, const QColor &colorSymbol,int& ind_ch ):
    data(dataH),ind_c(ind_ch)
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
    data(utilvec),ind_c(utilint)
{
    d_plot=d_plotH;
    setTitle(title);
    setPen(color,2);
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

    for(int i=0;i<(x.size());i++)
        if((i!=i1)&&(i!=i2))
            points<<QPointF(x[i],y[i]);

    setSamples( points ); // ассоциировать набор точек с кривой
    attach( d_plot); // отобразить кривую на графике
}
