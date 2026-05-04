#define _USE_MATH_DEFINES
#include <QPainterPath>
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "littlealgcpy.h"
#include "points.h"
#include "air_space_data.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <iostream>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    int size_of_window = 800;
    resize(size_of_window, size_of_window);
    setWindowTitle("Подзадача №4");
    setWindowFlags(windowFlags() & ~Qt::WindowMaximizeButtonHint);

    QGraphicsScene *workspace = new QGraphicsScene(this);
    workspace->setSceneRect(0, 0, size_of_window, size_of_window);
    workspace->setBackgroundBrush(Qt::white);

    QGraphicsView *view = new QGraphicsView(workspace, this);
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setCentralWidget(view);
    view->setRenderHint(QPainter::Antialiasing);

    QVector<Vec3D> points = {
        Vec3D(100, 743, 0),
        Vec3D(100, 100, 0),
        Vec3D(256, 300, 0),
        Vec3D(400, 700, 0),
        Vec3D(500, 600, 0),
        Vec3D(700, 700, 0),
        Vec3D(650, 200, 0),
        Vec3D(450, 300, 0),
        Vec3D(280, 450, 0)
    };

    QSet<BlockedAirCorridor> blockedPaths;

    QVector<Pvo> pvo_list;

    QVector<HighReliefZone> highreliefzone_list;




    Pvo pvo_main;
    pvo_main.id = 0;
    pvo_main.position = Vec3D(158, 521, 0);
    pvo_main.radius = 110;
    pvo_list.append(pvo_main);

    Pvo pvo_second;
    pvo_second.id = 1;
    pvo_second.position = Vec3D(350, 500, 0);
    pvo_second.radius = 70;
    pvo_list.append(pvo_second);

    Pvo pvo_third;
    pvo_third.id = 2;
    pvo_third.position = Vec3D(200, 200, 0);
    pvo_third.radius = 50;
    pvo_list.append(pvo_third);

    Pvo pvo_fourth;
    pvo_fourth.id = 3;
    pvo_fourth.position = Vec3D(300, 700, 0);
    pvo_fourth.radius = 50;
    pvo_list.append(pvo_fourth);

    HighReliefZone highreliefzone1;
    highreliefzone1.id = 1;
    highreliefzone1.vertices = {
        Vec3D(400, 743, 0),
        Vec3D(60, 700, 0),
        Vec3D(200, 690, 0)
    };
    highreliefzone_list.append(highreliefzone1);


    AirSpace airspace;
    airspace.points = points;
    airspace.air_corridors = blockedPaths;
    airspace.pvo_list = pvo_list;
    airspace.high_relief_zones = highreliefzone_list;

    float* distanceMatrix = createDistanceMatrix(airspace);
    int size = points.size();
    std::vector<int> result = LittleAlg(distanceMatrix, size);

    for(int i = 0; i < pvo_list.size(); ++i){
        drawCircle(pvo_list[i], workspace, size_of_window);
    }

    for(int i = 0; i < highreliefzone_list.size(); ++i){
        drawPolygon(highreliefzone_list[i], workspace, size_of_window);
    }

    for(int index = 0; index < result.size() - 1; ++index){
        drawPathWithObstacles(points[result[index]], points[result[index + 1]],
                       airspace, workspace, size_of_window);
    }
    drawPathWithObstacles(points[result[size - 1]], points[result[0]],
                   airspace, workspace, size_of_window);

    for(int i = 0; i < size; ++i){
        drawPoint(points[i], workspace, size_of_window, i);
    }

    delete[] distanceMatrix;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::drawPoint(Vec3D point, QGraphicsScene* scene, int windowHeight, int index)
{
    scene->addEllipse(point.x - 4, windowHeight - point.y - 4, 8, 8,
                      QPen(Qt::black, 2), QBrush(Qt::red));

    QGraphicsTextItem* text = scene->addText(QString::number(index));
    text->setFont(QFont("Arial", 9, QFont::Bold));
    text->setDefaultTextColor(Qt::black);
    text->setPos(point.x + 8, windowHeight - point.y - 8);
}

void MainWindow::drawLine(Vec3D p1, Vec3D p2, QGraphicsScene* scene, int windowHeight)
{
    int y1_qt = windowHeight - p1.y;
    int y2_qt = windowHeight - p2.y;
    scene->addLine(p1.x, y1_qt, p2.x, y2_qt, QPen(Qt::black, 2));
}

void MainWindow::drawCircle(Pvo pvo, QGraphicsScene* scene, int windowHeight)
{
    int centerY = windowHeight - pvo.position.y;
    int radius = pvo.radius;

    QPen pen(Qt::red, 2);
    pen.setStyle(Qt::DashLine);
    QBrush brush(QColor(255, 100, 100, 50));
    scene->addEllipse(pvo.position.x - radius, centerY - radius,
                      radius * 2, radius * 2, pen, brush);

    scene->addEllipse(pvo.position.x - 3, centerY - 3, 6, 6,
                      QPen(Qt::red, 1), QBrush(Qt::red));
}

void MainWindow::drawPathWithObstacles(const Vec3D& p1, const Vec3D& p2,
                                        const AirSpace& airspace,
                                        QGraphicsScene* scene, int windowHeight)
{
    for(const auto& zone : airspace.high_relief_zones) {
        if(segmentIntersectsPolygon(p1, p2, zone)) {
            BypassPath bypass = calculatePolygonBypass(p1, p2, zone);
            if(bypass.waypoints.size() >= 2) {
                // Рекурсивно обходим препятствия на каждой секции обхода
                for(int i = 0; i < bypass.waypoints.size() - 1; ++i) {
                    bool hasIntersection = false;

                    for(const auto& pvo : airspace.pvo_list) {
                        double distToSegment = distanceToSegment(pvo.position,
                                            bypass.waypoints[i], bypass.waypoints[i + 1]);
                        if(distToSegment <= pvo.radius) {
                            hasIntersection = true;
                            BypassPath circleBypass = calculateCircularBypass(
                                bypass.waypoints[i], bypass.waypoints[i + 1],
                                pvo.position, pvo.radius);
                            if(circleBypass.waypoints.size() >= 4) {
                                drawLine(circleBypass.waypoints[0], circleBypass.waypoints[1],
                                        scene, windowHeight);
                                drawLine(circleBypass.waypoints[2], circleBypass.waypoints[3],
                                        scene, windowHeight);

                                double angle1 = atan2(circleBypass.waypoints[1].y - pvo.position.y,
                                                      circleBypass.waypoints[1].x - pvo.position.x);
                                double angle2 = atan2(circleBypass.waypoints[2].y - pvo.position.y,
                                                      circleBypass.waypoints[2].x - pvo.position.x);
                                if(angle1 < 0) angle1 += 2*M_PI;
                                if(angle2 < 0) angle2 += 2*M_PI;

                                double delta = angle2 - angle1;
                                if(delta > M_PI) delta -= 2*M_PI;
                                if(delta < -M_PI) delta += 2*M_PI;

                                QPainterPath path;
                                QRectF rect(pvo.position.x - pvo.radius,
                                           windowHeight - pvo.position.y - pvo.radius,
                                           pvo.radius * 2, pvo.radius * 2);
                                path.arcMoveTo(rect, angle1 * 180 / M_PI);
                                path.arcTo(rect, angle1 * 180 / M_PI, delta * 180 / M_PI);
                                scene->addPath(path, QPen(Qt::black, 2));
                            }
                            break;
                        }
                    }

                    if(!hasIntersection) {
                        drawLine(bypass.waypoints[i], bypass.waypoints[i + 1],
                                scene, windowHeight);
                    }
                }
            }
            return;
        }
    }

    for(const auto& pvo : airspace.pvo_list) {
        double distToSegment = distanceToSegment(pvo.position, p1, p2);
        if(distToSegment <= pvo.radius) {
            BypassPath bypass = calculateCircularBypass(p1, p2, pvo.position, pvo.radius);
            if(bypass.waypoints.size() >= 4) {
                drawLine(bypass.waypoints[0], bypass.waypoints[1], scene, windowHeight);
                drawLine(bypass.waypoints[2], bypass.waypoints[3], scene, windowHeight);

                double angle1 = atan2(bypass.waypoints[1].y - pvo.position.y,
                                      bypass.waypoints[1].x - pvo.position.x);
                double angle2 = atan2(bypass.waypoints[2].y - pvo.position.y,
                                      bypass.waypoints[2].x - pvo.position.x);
                if(angle1 < 0) angle1 += 2*M_PI;
                if(angle2 < 0) angle2 += 2*M_PI;

                double delta = angle2 - angle1;
                if(delta > M_PI) delta -= 2*M_PI;
                if(delta < -M_PI) delta += 2*M_PI;

                QPainterPath path;
                QRectF rect(pvo.position.x - pvo.radius,
                           windowHeight - pvo.position.y - pvo.radius,
                           pvo.radius * 2, pvo.radius * 2);
                path.arcMoveTo(rect, angle1 * 180 / M_PI);
                path.arcTo(rect, angle1 * 180 / M_PI, delta * 180 / M_PI);
                scene->addPath(path, QPen(Qt::black, 2));
            }
            return;
        }
    }

    drawLine(p1, p2, scene, windowHeight);
}


void MainWindow::drawPolygon(HighReliefZone highreliefzone, QGraphicsScene* scene, int windowHeight)
{
    QVector<Vec3D> points = highreliefzone.vertices;

    if (points.size() < 3) return;

    QPen pen(Qt::blue, 2);
    pen.setStyle(Qt::DashLine);

    int y1_qt, y2_qt;

    for(int i = 0; i < points.size() - 1; ++i){
        y1_qt = windowHeight - points[i].y;
        y2_qt = windowHeight - points[i+1].y;
        scene->addLine(points[i].x, y1_qt, points[i+1].x, y2_qt, pen);
    }
    y1_qt = windowHeight - points[points.size() - 1].y;
    y2_qt = windowHeight - points[0].y;
    scene->addLine(points[points.size() - 1].x, y1_qt, points[0].x, y2_qt, pen);
}
