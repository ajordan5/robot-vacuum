#ifndef MAPPAINTER_H
#define MAPPAINTER_H

#include <QWidget>
#include <QPainter>
#include <lidarmapper.h>

class MapPainter : public QWidget
{
    Q_OBJECT

public:
    MapPainter(QWidget* parent = 0);

public slots:
    void save_map_pointer(LidarMapper* map);

protected:
    void paintEvent(QPaintEvent* event);
    const uchar* mapImage;
    int mapWidth;
    int mapHeight;
    int numberOfBytesPerWidth;

};

#endif // MAPPAINTER_H
