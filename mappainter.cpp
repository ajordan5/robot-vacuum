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
    QImage image{mapImage,mapWidth,mapHeight,numberOfBytesPerWidth,QImage::Format_ARGB32};
    painter.drawImage(QRect{0,0,this->width(),this->height()},image.mirrored());

}

void MapPainter::save_map_pointer(LidarMapper* map)
{
    mapImage = map->get_image();
    mapWidth = map->get_num_cols();
    mapHeight = map->get_num_rows();

    int intSize{sizeof(int)};
    numberOfBytesPerWidth = mapWidth*intSize;


}
