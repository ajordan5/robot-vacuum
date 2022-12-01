#include "mappainter.h"
#include <QDebug>

MapPainter::MapPainter(QWidget* parent)
    : QWidget(parent)
{

}

void MapPainter::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    int width{1000};
    int height{1000};
    int intSize{sizeof(int)};
    int numberOfBytesPerWidth{width*intSize};
    QImage image{mapImage,width,height,numberOfBytesPerWidth,QImage::Format_ARGB32};
    painter.drawImage(QRect{0,0,this->width(),this->height()},image.mirrored());

}

void MapPainter::save_map_pointer(const uchar* map)
{
    mapImage = map;

}
