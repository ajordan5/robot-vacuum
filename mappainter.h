#ifndef MAPPAINTER_H
#define MAPPAINTER_H

#include <QWidget>
#include <QPainter>

class MapPainter : public QWidget
{
    Q_OBJECT

public:
    MapPainter(QWidget* parent = 0);

public slots:
    void save_map_pointer(const uchar* map);

protected:
    void paintEvent(QPaintEvent* event);
    const uchar* mapImage;

};

#endif // MAPPAINTER_H
