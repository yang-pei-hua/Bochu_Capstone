#include "ui/panels/FeatureDetailsPanel.h"
#include "ui/panels/FeatureEditorPanel.h"
#include "core/SketchFrame.h"
#include <modeling/SemanticFaces.h>

#include <QApplication>
#include <QAction>
#include <QClipboard>
#include <QMenu>
#include <QTextBrowser>
#include <QUrl>
#include <QVBoxLayout>

#include <cmath>
#include <optional>
#include <type_traits>

namespace {
using namespace modeling;
QString number(double value) { return QString::number(value == 0 ? 0 : value, 'g', 12); }
QString vector(double x, double y, double z)
{
    return QStringLiteral("(%1, %2, %3)").arg(number(x), number(y), number(z));
}
QString vector(const Vec3& v) { return vector(v.x, v.y, v.z); }
QString vector(const double v[3]) { return vector(v[0], v[1], v[2]); }
QString point(double u, double v) { return QStringLiteral("(%1, %2)").arg(number(u), number(v)); }
QString worldPoint(const sketchapp::PlaneFrame& f, double u, double v)
{
    return vector(f.origin[0] + u*f.xDirection[0] + v*f.yDirection[0],
                  f.origin[1] + u*f.xDirection[1] + v*f.yDirection[1],
                  f.origin[2] + u*f.xDirection[2] + v*f.yDirection[2]);
}
const Feature* featureById(const std::vector<Feature>& features, FeatureId id)
{
    for (const auto& f : features) if (f.id == id) return &f;
    return nullptr;
}
QString reference(const std::vector<Feature>& features, FeatureId id)
{
    const auto* f = featureById(features, id);
    if (!f) return QStringLiteral("#%1（引用不存在）").arg(id);
    return QStringLiteral("<a href=\"feature:%1\">%2 (#%1)</a>")
        .arg(id).arg(QString::fromStdString(f->name).toHtmlEscaped());
}
QString operation(ExtrudeOperation op)
{
    switch (op) {
    case ExtrudeOperation::NewBody: return QStringLiteral("新实体（NewBody）");
    case ExtrudeOperation::Join: return QStringLiteral("合并（Join）");
    case ExtrudeOperation::Cut: return QStringLiteral("切除（Cut）");
    case ExtrudeOperation::Intersect: return QStringLiteral("求交（Intersect）");
    }
    return QStringLiteral("未知");
}
bool resolvedFrame(const std::vector<Feature>& features, const SketchPlaneReference& plane,
                   sketchapp::PlaneFrame& frame)
{
    if (const auto* p = std::get_if<FaceReference>(&plane))
        if (p->role == FaceRole::Unknown) return false;
    if (const auto* p = std::get_if<Plane3d>(&plane)) {
        const auto& n = p->normal; const auto& x = p->xDirection;
        const double crossLength = std::hypot(n.y*x.z-n.z*x.y, n.z*x.x-n.x*x.z, n.x*x.y-n.y*x.x);
        if (!std::isfinite(crossLength) || crossLength <= 1e-12) return false;
    }
    return sketchapp::resolveSketchFrame(features,plane,frame);
}
struct Details {
    QString html;
    void section(const QString& text) { html += "</table><h4>" + text.toHtmlEscaped() + "</h4><table cellspacing=\"6\">"; }
    void row(const QString& label, const QString& value, bool rich = false) {
        html += "<tr><td valign=\"top\">" + label.toHtmlEscaped()
            + "</td><td>" + (rich ? value : value.toHtmlEscaped().replace("\n","<br>")) + "</td></tr>";
    }
    void note(const QString& text) { html += "</table><p>" + text.toHtmlEscaped() + "</p><table cellspacing=\"6\">"; }
    void length(const QString& label, double value) { row(label, number(value) + " mm"); }
    void frame(const sketchapp::PlaneFrame& f) {
        row("原点 O（mm）", vector(f.origin));
        row("局部 X / U 轴", vector(f.xDirection));
        row("局部 Y / V 轴", vector(f.yDirection));
        row("法向 N", vector(f.normal));
    }
    std::optional<sketchapp::PlaneFrame> sketch(const std::vector<Feature>& features,
                                              const SketchFeatureParams& params) {
        section("草图平面与坐标系");
        row("平面", sketchapp::planeLabel(params.plane));
        if (const auto* p = std::get_if<OffsetDatumPlane>(&params.plane)) {
            row("基准平面", sketchapp::planeLabel(datumPlane(p->base)));
            length("沿基准法向偏移", p->offset);
        }
        if (const auto* p = std::get_if<FaceReference>(&params.plane)) {
            row("依附特征", reference(features, p->ownerFeature), true);
            row("依附面", QString::fromLatin1(modeling::faceRoleName(p->role)));
        }
        sketchapp::PlaneFrame f;
        const bool resolved = resolvedFrame(features, params.plane, f);
        if (resolved) frame(f);
        else row("坐标系", "平面引用无法解析，无法确定世界坐标。");
        note("草图 (U,V) 使用毫米。世界点 P = O + U·X轴 + V·Y轴。");
        section(QStringLiteral("草图轮廓（%1 个元素）").arg(params.entities.size()));
        for (const auto& e : params.entities) {
            const auto prefix = QStringLiteral("#%1 ").arg(e.id);
            const auto location = [&](double u, double v) {
                return point(u,v) + (resolved ? " → " + worldPoint(f,u,v) : QString());
            };
            std::visit([&](const auto& g) {
                using T = std::decay_t<decltype(g)>;
                if constexpr (std::is_same_v<T, Point2D>) row(prefix + "点", location(g.x,g.y));
                else if constexpr (std::is_same_v<T, Line2D>) {
                    row(prefix + "线段起点", location(g.x1,g.y1));
                    row(prefix + "线段终点", location(g.x2,g.y2));
                    length(prefix + "线段长度", std::hypot(g.x2-g.x1,g.y2-g.y1));
                } else if constexpr (std::is_same_v<T, Rectangle2D>) {
                    row(prefix + "矩形起点", location(g.x,g.y));
                    row(prefix + "矩形尺寸", point(g.width,g.height) + " mm");
                    row(prefix + "其他角点", location(g.x+g.width,g.y) + "\n"
                        + location(g.x+g.width,g.y+g.height) + "\n" + location(g.x,g.y+g.height));
                } else {
                    row(prefix + "圆心", location(g.x,g.y));
                    length(prefix + "半径",g.radius); length(prefix + "直径",2*g.radius);
                }
            },e.geometry);
        }
        if (params.entities.empty()) row("轮廓", "尚无草图元素");
        return resolved ? std::optional<sketchapp::PlaneFrame>(f) : std::nullopt;
    }
};
}

FeatureDetailsPanel::FeatureDetailsPanel(QWidget* parent) : QWidget(parent)
{
    setObjectName("featureDetailsPanel");
    auto* layout = new QVBoxLayout(this); layout->setContentsMargins(6,6,6,6);
    m_editor = new FeatureEditorPanel(this);
    layout->addWidget(m_editor);
    m_details = new QTextBrowser(this); m_details->setObjectName("featureDetailsText");
    m_details->setOpenLinks(false);
    layout->addWidget(m_details);
    connect(m_details, &QTextBrowser::anchorClicked, this, [this](const QUrl& url) {
        if (url.scheme() != "feature") return;
        bool ok = false; const auto id = url.path().toULongLong(&ok);
        if (ok) emit featureRequested(id);
    });
    m_details->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_details, &QTextBrowser::customContextMenuRequested, this, [this](const QPoint& pos) {
        auto* menu = m_details->createStandardContextMenu();
        menu->addSeparator();
        auto* copy = menu->addAction(tr("复制全部特征详情"));
        copy->setObjectName("copyFeatureDetailsAction");
        connect(copy, &QAction::triggered, this, [this] {
            QApplication::clipboard()->setText(m_details->toPlainText());
        });
        menu->exec(m_details->viewport()->mapToGlobal(pos)); delete menu;
    });
    setFeature({}, kInvalidFeatureId);
}

void FeatureDetailsPanel::setFeature(const std::vector<Feature>& features, FeatureId id)
{
    const auto* feature = featureById(features, id);
    if (!feature) {
        m_details->setPlainText(tr("在左侧 CAD 特征列表中选择一项，查看尺寸、坐标系和建模参数。"));
        return;
    }
    Details d;
    d.html = "<h3>" + QString::fromStdString(feature->name).toHtmlEscaped() + "</h3><table cellspacing=\"6\">";
    d.row("特征 ID", QString::number(feature->id));
    d.row("来源", feature->reconstructed ? "点云重建" : "手动建模");
    if (feature->manuallyEdited) d.row("调整状态", "已手动修改；重拟合覆盖需要确认");
    d.row("状态", feature->suppressed ? "已抑制" : feature->valid ? "有效" : "无效");
    if (!feature->errorText.empty()) d.row("错误", QString::fromStdString(feature->errorText));
    d.note("尺寸与坐标单位：mm；方向向量无单位。坐标属于建模空间，未叠加 Object 面板的显示变换。");
    std::visit([&](const auto& p) {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, BoxPrimitiveParams>) {
            d.section("长方体参数");
            d.length("局部 X 长度",p.sizeX); d.length("局部 Y 长度",p.sizeY); d.length("局部 Z 长度",p.sizeZ);
            d.row("融合设置",p.join ? "已有实体时合并（Join）；首个实体直接创建" : "创建独立实体（NewBody）");
            auto f = sketchapp::planeFrameOf({p.pose.origin,p.pose.zDirection,p.pose.xDirection});
            d.section("长方体坐标系"); d.frame(f);
            d.section("等价草图与拉伸步骤");
            d.row("草图平面", "过原点 O，法向为 N，草图 U/V 轴为局部 X/Y 轴。");
            d.row("矩形角点（U,V）", point(0,0) + "；" + point(p.sizeX,0) + "；"
                + point(p.sizeX,p.sizeY) + "；" + point(0,p.sizeY));
            d.length("沿 +N 拉伸",p.sizeZ);
            d.note("这是该长方体的等价建模方法。组合长方体可能重叠，不能把各自体积直接相加。");
        } else if constexpr (std::is_same_v<T, SketchFeatureParams>) {
            d.sketch(features,p);
        } else if constexpr (std::is_same_v<T, ExtrudeFeatureParams> || std::is_same_v<T, CutFeatureParams>) {
            constexpr bool cut = std::is_same_v<T, CutFeatureParams>;
            d.section(cut ? "切除参数" : "拉伸参数");
            d.row("来源草图", reference(features,p.sketchId),true);
            if constexpr (cut) {
                d.row("终止方式",p.throughAll ? "贯穿全部（沿切除方向）" : "给定深度");
                if (!p.throughAll) d.length("深度",p.depth);
                d.row("运算", "切除（Cut）");
            } else { d.length("深度",p.depth); d.row("运算",operation(p.operation)); }
            const bool negative = cut ? !p.reverse : p.reverse;
            d.row("方向",negative ? "沿草图法向 -N" : "沿草图法向 +N");
            const auto* source = featureById(features,p.sketchId);
            const auto* sketch = source ? std::get_if<SketchFeatureParams>(&source->params) : nullptr;
            if (sketch) {
                sketchapp::PlaneFrame f;
                if (resolvedFrame(features,sketch->plane,f)) {
                    const double sign = negative ? -1.0 : 1.0;
                    d.row("世界方向",vector(sign*f.normal[0],sign*f.normal[1],sign*f.normal[2]));
                }
                d.sketch(features,*sketch);
            } else d.row("草图状态","来源草图不存在或类型不匹配，无法显示轮廓。");
        } else if constexpr (std::is_same_v<T, ThroughHolePrimitiveParams>) {
            d.section("贯穿孔参数"); d.row("轴线上一点（mm）",vector(p.axisPoint));
            d.row("轴向",vector(p.axisDirection)); d.length("半径",p.radius); d.length("直径",2*p.radius);
            d.row("运算","沿轴线双向贯穿当前实体（Cut）");
        } else if constexpr (std::is_same_v<T, SpherePrimitiveParams>) {
            d.section("球体参数"); d.row("球心（mm）",vector(p.center));
            d.length("半径",p.radius); d.length("直径",2*p.radius);
        } else if constexpr (std::is_same_v<T, CylinderPrimitiveParams>) {
            d.section("圆柱参数"); d.row("底面中心（mm）",vector(p.baseCenter)); d.row("轴向",vector(p.axisDirection));
            d.length("半径",p.radius); d.length("直径",2*p.radius); d.length("高度",p.height);
        } else if constexpr (std::is_same_v<T, ConePrimitiveParams>) {
            d.section("圆锥 / 圆台参数"); d.row("起始面中心（mm）",vector(p.baseCenter)); d.row("轴向",vector(p.axisDirection));
            d.length("起始半径",p.baseRadius); d.length("终止半径",p.topRadius); d.length("高度",p.height);
        } else if constexpr (std::is_same_v<T, TorusPrimitiveParams>) {
            d.section("圆环体参数"); d.row("中心（mm）",vector(p.center)); d.row("轴向",vector(p.axisDirection));
            d.length("主半径",p.majorRadius); d.length("管截面半径",p.minorRadius);
        }
    },feature->params);
    m_details->setHtml(d.html + "</table>");
}
