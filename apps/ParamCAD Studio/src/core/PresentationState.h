#pragma once
#include <QColor>
#include <QString>
#include <array>

struct ObjectPresentation
{
    std::array<double, 3> position{0, 0, 0}, rotation{0, 0, 0}, scale{1, 1, 1};
    bool visible = true;
    int representation = 0;
    double opacity = 1;
    QColor color{71, 145, 214};
};
struct RenderPresentation
{
    QColor background{26, 28, 33};
    QColor background2{43, 46, 54};
    bool gradientBackground = true;
    bool lighting = true, textureEnabled = false;
    int width = 1920, height = 1080, textureProjection = 0;
    QString texturePath;
};
