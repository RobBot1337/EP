#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include "air_space_data.h"
#include "vec3D.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    void drawPoint(Vec3D point, QGraphicsScene* scene, int windowHeight, int index);
    void drawLine(Vec3D p1, Vec3D p2, QGraphicsScene* scene, int windowHeight, QColor color = Qt::black);
    void drawCircle(Pvo pvo, QGraphicsScene* scene, int windowHeight);
    void drawPathWithPVO(const Vec3D& p1, const Vec3D& p2, const AirSpace& airspace,
                        QGraphicsScene* scene, int windowHeight);
    void drawArc(const Vec3D& center, double radius, const Vec3D& start,
                 const Vec3D& end, QGraphicsScene* scene, int windowHeight);
    void drawPolygon(HighReliefZone highreliefzone, QGraphicsScene* scene, int windowHeight);
    void drawPathWithObstacles(const Vec3D& p1, const Vec3D& p2, const AirSpace& airspace,
                               QGraphicsScene* scene, int windowHeight, QColor color = Qt::black);
};

#endif
