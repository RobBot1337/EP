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
    setWindowTitle("Проект");
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
        Vec3D(280, 450, 0),
        Vec3D(500, 500, 0)
    };

    QSet<BlockedAirCorridor> blockedPaths;

    QVector<Pvo> pvo_list;

    QVector<HighReliefZone> highreliefzone_list;

    Pvo pvo_main;
    pvo_main.id = 0;
    pvo_main.position = Vec3D(158, 521, 0);
    pvo_main.radius = 110;
    pvo_list.append(pvo_main);

    Pvo pvo_2;
    pvo_2.id = 1;
    pvo_2.position = Vec3D(200, 200, 0);
    pvo_2.radius = 70;
    pvo_list.append(pvo_2);



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


    int numSalesmen = 3;
    int originalSize = points.size();

    QVector<QColor> colors = {
        Qt::red, Qt::green, Qt::blue, Qt::magenta,
        Qt::cyan, Qt::darkYellow, Qt::darkMagenta, Qt::darkCyan,
        Qt::darkRed, Qt::darkGreen, Qt::darkBlue
    };

    std::vector<std::vector<int>> tours = solveGreedyMTSP(airspace, numSalesmen);

    // Рисуем препятствия
    for(int i = 0; i < pvo_list.size(); ++i){
        drawCircle(pvo_list[i], workspace, size_of_window);
    }

    for(int i = 0; i < highreliefzone_list.size(); ++i){
        drawPolygon(highreliefzone_list[i], workspace, size_of_window);
    }

    // Рисуем маршруты коммивояжёров разными цветами
    for (int s = 0; s < tours.size(); ++s) {
        const auto& tour = tours[s];
        if (tour.size() < 2) continue;

        QColor tourColor = colors[s % colors.size()];

        // Рисуем путь между точками в маршруте
        for (int i = 0; i < tour.size() - 1; ++i) {
            drawPathWithObstacles(points[tour[i]], points[tour[i + 1]],
                                 airspace, workspace, size_of_window, tourColor);
        }
    }

    // Рисуем точки
    for(int i = 0; i < originalSize; ++i){
        drawPoint(points[i], workspace, size_of_window, i);
    }
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

void MainWindow::drawLine(Vec3D p1, Vec3D p2, QGraphicsScene* scene, int windowHeight, QColor color)
{
    int y1_qt = windowHeight - p1.y;
    int y2_qt = windowHeight - p2.y;
    scene->addLine(p1.x, y1_qt, p2.x, y2_qt, QPen(color, 2));
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
                                        QGraphicsScene* scene, int windowHeight,
                                        QColor color)
{
    for(const auto& zone : airspace.high_relief_zones) {
        if(segmentIntersectsPolygon(p1, p2, zone)) {
            BypassPath bypass = calculatePolygonBypass(p1, p2, zone);
            if(bypass.waypoints.size() >= 2) {
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
                                        scene, windowHeight, color);
                                drawLine(circleBypass.waypoints[2], circleBypass.waypoints[3],
                                        scene, windowHeight, color);

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
                                scene->addPath(path, QPen(color, 2));
                            }
                            break;
                        }
                    }

                    if(!hasIntersection) {
                        drawLine(bypass.waypoints[i], bypass.waypoints[i + 1],
                                scene, windowHeight, color);
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
                drawLine(bypass.waypoints[0], bypass.waypoints[1], scene, windowHeight, color);
                drawLine(bypass.waypoints[2], bypass.waypoints[3], scene, windowHeight, color);

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
                scene->addPath(path, QPen(color, 2));
            }
            return;
        }
    }

    drawLine(p1, p2, scene, windowHeight, color);
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
