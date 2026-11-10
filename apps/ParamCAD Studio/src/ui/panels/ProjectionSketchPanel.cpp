#include "ui/panels/ProjectionSketchPanel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QPromise>
#include <QPushButton>
#include <QThreadPool>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>

namespace {
using reconstruction::ProjectionSketchBoundary;
using reconstruction::ProjectionSketchInput;

const QColor outerColor(255, 173, 72);
const QColor holeColor(240, 128, 224);

class SketchCanvas final : public QWidget
{
public:
    explicit SketchCanvas(QWidget* parent) : QWidget(parent)
    {
        setObjectName(QStringLiteral("projectionSketchCanvas"));
        setMinimumHeight(155);
        setToolTip(QObject::tr("滚轮缩放，拖动平移，双击复位。"));
    }

    void setInput(const ProjectionSketchInput& input)
    {
        m_points.clear();
        m_points.reserve(qsizetype(input.points.size()));
        for (const auto& point : input.points) m_points.push_back({point.x, point.y});
        m_bounds = m_points.boundingRect();
        m_boundary = {};
        resetView();
    }

    void setBoundary(ProjectionSketchBoundary boundary)
    {
        m_boundary = std::move(boundary);
        update();
    }

    void setPointsVisible(bool visible)
    {
        m_pointsVisible = visible;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        const QRectF viewport = plotRect();
        painter.fillRect(rect(), QColor(20, 28, 39));
        painter.setPen(QPen(QColor(109, 129, 153), 2));
        painter.drawRect(QRectF(rect()).adjusted(1, 1, -1, -1));
        painter.setPen(QColor(168, 188, 210));
        painter.drawText(QRectF(12, height()-22, width()-24, 20),
                         Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("u →   v ↑   mm"));
        if (m_points.isEmpty() || viewport.isEmpty()) return;
        painter.setClipRect(viewport.adjusted(1, 1, -1, -1));
        const double scale = .90*m_zoom*std::min(viewport.width()/std::max(m_bounds.width(), 1e-9),
                                               viewport.height()/std::max(m_bounds.height(), 1e-9));
        painter.translate(viewport.center()+m_pan);
        painter.scale(scale,-scale);
        painter.translate(-m_bounds.center());
        const auto pen = [](QColor color, qreal width) {
            QPen result(color,width,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
            result.setCosmetic(true);
            return result;
        };
        if (m_pointsVisible) {
            painter.setPen(pen(QColor(79,155,218,180),1.5));
            painter.drawPoints(m_points);
        }

        painter.setRenderHint(QPainter::Antialiasing);
        const auto drawRing = [&](const auto& ring, QColor color) {
            if (ring.empty()) return;
            QPolygonF polygon;
            polygon.reserve(qsizetype(ring.size()));
            for (const auto& point : ring) polygon.push_back({point.x,point.y});
            painter.setPen(pen(color,2.2));
            painter.drawPolygon(polygon);
            const auto bounds = polygon.boundingRect();
            if (scale*std::max(bounds.width(),bounds.height()) >= 4) return;
            // Keep subpixel rings visible at overview scale with a screen-sized cross.
            const auto center = bounds.center();
            const double reach = 3/scale;
            painter.drawLine(center-QPointF(reach,0),center+QPointF(reach,0));
            painter.drawLine(center-QPointF(0,reach),center+QPointF(0,reach));
        };
        for (const auto& region : m_boundary.regions) {
            drawRing(region.outer,outerColor);
            for (const auto& hole : region.holes) drawRing(hole,holeColor);
        }
    }

    void wheelEvent(QWheelEvent* event) override
    {
        const double zoom = std::clamp(m_zoom*std::pow(1.25,event->angleDelta().y()/120.),1.,256.);
        const QPointF anchor = event->position()-plotRect().center();
        m_pan = anchor + (m_pan-anchor)*(zoom/m_zoom);
        m_zoom = zoom;
        if (m_zoom == 1) m_pan = {};
        update();
        event->accept();
    }

    void mousePressEvent(QMouseEvent* event) override { m_lastPosition = event->position(); }
    void mouseMoveEvent(QMouseEvent* event) override
    {
        if (!(event->buttons() & Qt::LeftButton)) return;
        m_pan += event->position()-m_lastPosition;
        m_lastPosition = event->position();
        update();
    }
    void mouseDoubleClickEvent(QMouseEvent*) override { resetView(); }

private:
    QRectF plotRect() const { return QRectF(rect()).adjusted(12, 12, -12, -24); }

    void resetView()
    {
        m_zoom = 1;
        m_pan = {};
        update();
    }

    QPolygonF m_points;
    QRectF m_bounds;
    ProjectionSketchBoundary m_boundary;
    QPointF m_pan;
    QPointF m_lastPosition;
    double m_zoom = 1;
    bool m_pointsVisible = true;
};

QString inputLabel(const ProjectionSketchInput& input)
{
    using Section = ProjectionSketchInput::Section;
    const auto& n = input.plane.normal;
    const auto section = input.section == Section::FirstCap ? QObject::tr("端面 A")
        : input.section == Section::LastCap ? QObject::tr("端面 B") : QObject::tr("整件投影");
    return QObject::tr("%1 · 轴 (%2, %3, %4)").arg(section)
        .arg(n.x,0,'f',3).arg(n.y,0,'f',3).arg(n.z,0,'f',3);
}

int preferredInput(const std::vector<ProjectionSketchInput>& inputs,
                   const reconstruction::ProjectionOperation* operation)
{
    if (!operation) return 0;
    const auto* plane = std::get_if<modeling::Plane3d>(&operation->sketch.plane);
    if (!plane) return 0;
    const auto match = std::find_if(inputs.begin(),inputs.end(),[&](const auto& input) {
        const auto& n = input.plane.normal;
        const auto& axis = plane->normal;
        if (std::hypot(n.x-axis.x,n.y-axis.y,n.z-axis.z) > 1e-7) return false;
        if (input.section == ProjectionSketchInput::Section::WholePart) return true;
        return !operation->supportPointIds.empty() && input.firstPointId == operation->supportPointIds.front()
            && input.points.size() == operation->supportPointIds.size();
    });
    return match == inputs.end() ? 0 : int(match-inputs.begin());
}
}

ProjectionSketchPanel::ProjectionSketchPanel(std::vector<ProjectionSketchInput> inputs,
    const reconstruction::ProjectionOperation* preferred, QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    auto* axes = new QComboBox(this);
    axes->setObjectName(QStringLiteral("projectionSketchInput"));
    for (const auto& input : inputs) axes->addItem(inputLabel(input));
    axes->setCurrentIndex(preferredInput(inputs,preferred));
    auto* fit = new QPushButton(QObject::tr("拟合边界"),this);
    fit->setObjectName(QStringLiteral("fitProjectionBoundaryButton"));
    auto* controls = new QHBoxLayout;
    controls->addWidget(axes,1);
    controls->addWidget(fit);
    layout->addLayout(controls);
    auto* canvas = new SketchCanvas(this);
    layout->addWidget(canvas,1);
    auto* display = new QHBoxLayout;
    auto* showPoints = new QCheckBox(QObject::tr("显示散点"),this);
    showPoints->setObjectName(QStringLiteral("showProjectionSketchPoints"));
    showPoints->setChecked(true);
    connect(showPoints,&QCheckBox::toggled,canvas,&SketchCanvas::setPointsVisible);
    display->addWidget(showPoints);
    auto* legend = new QLabel(
        QStringLiteral("<span style='color:%1'>%2</span> &nbsp; <span style='color:%3'>%4</span>")
            .arg(outerColor.name(),QObject::tr("外环"),holeColor.name(),QObject::tr("孔")),this);
    legend->setTextFormat(Qt::RichText);
    legend->setToolTip(QObject::tr("缩小时，十字标记表示尺寸很小的轮廓。"));
    display->addWidget(legend);
    display->addStretch();
    layout->addLayout(display);
    auto* status = new QLabel(this);
    status->setObjectName(QStringLiteral("projectionSketchStatus"));
    status->setWordWrap(true);
    layout->addWidget(status);
    const auto sketches = std::make_shared<const std::vector<ProjectionSketchInput>>(std::move(inputs));
    const auto refresh = [canvas,status,fit,sketches](int index) {
        if (index < 0) {
            fit->setEnabled(false);
            status->setText(QObject::tr("当前方式没有可显示的二维点。"));
            return;
        }
        const auto& input = sketches->at(std::size_t(index));
        canvas->setInput(input);
        fit->setEnabled(true);
        status->setText(QObject::tr("%1 个投影点 · 滚轮缩放，拖动平移，双击复位")
                            .arg(qulonglong(input.points.size())));
    };
    connect(axes,&QComboBox::currentIndexChanged,this,refresh);
    refresh(axes->currentIndex());
    connect(fit,&QPushButton::clicked,this,[this,axes,fit,canvas,status,sketches] {
        const int index = axes->currentIndex();
        if (index < 0) return;
        axes->setEnabled(false);
        fit->setEnabled(false);
        status->setText(QObject::tr("正在拟合二维边界…"));
        auto promise = std::make_shared<QPromise<ProjectionSketchBoundary>>();
        auto* watcher = new QFutureWatcher<ProjectionSketchBoundary>(this);
        connect(watcher,&QFutureWatcherBase::finished,this,[watcher,axes,fit,canvas,status,sketches,index] {
            try {
                auto boundary = watcher->result();
                std::size_t holes = 0;
                for (const auto& region : boundary.regions) holes += region.holes.size();
                auto summary = QObject::tr("%1 个点 · %2 个外环（橙） · %3 个内环（紫） · Alpha 半径 %4 mm")
                                    .arg(qulonglong(sketches->at(std::size_t(index)).points.size()))
                                    .arg(qulonglong(boundary.regions.size())).arg(qulonglong(holes))
                                    .arg(boundary.alphaRadius,0,'g',5);
                if (boundary.discardedRegionCount)
                    summary += QObject::tr(" · 筛掉 %1 个低支持区域").arg(qulonglong(boundary.discardedRegionCount));
                status->setText(summary);
                canvas->setBoundary(std::move(boundary));
            } catch (...) {
                status->setText(QObject::tr("二维边界拟合失败。"));
            }
            watcher->deleteLater();
            axes->setEnabled(true);
            fit->setEnabled(true);
        });
        promise->start();
        watcher->setFuture(promise->future());
        QThreadPool::globalInstance()->start([promise,sketches,index] {
            try {
                promise->addResult(reconstruction::fitProjectionSketchBoundary(sketches->at(std::size_t(index))));
            } catch (...) {
                promise->setException(std::current_exception());
            }
            promise->finish();
        });
    });
}
