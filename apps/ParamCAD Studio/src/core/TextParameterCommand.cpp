#include "core/TextParameterCommand.h"
#include <modeling/PrimitiveEditing.h>
#include <QRegularExpression>
#include <QStringList>
#include <cmath>
#include <set>

namespace textcad {
namespace {
using namespace modeling;
QString number(double value) { return QString::number(value, 'g', 12); }
QString anchorName(ResizeAnchor anchor) {
    switch (anchor) {
    case ResizeAnchor::Center: return QStringLiteral("中心");
    case ResizeAnchor::Base: return QStringLiteral("底面中心");
    case ResizeAnchor::Top: return QStringLiteral("顶面中心");
    default: {
        const QStringList sides{"局部 X− 面", "局部 X+ 面", "局部 Y− 面", "局部 Y+ 面", "局部 Z− 面", "局部 Z+ 面"};
        const int side = static_cast<int>(anchor) - 1;
        return side >= 0 && side < sides.size() ? sides[side] : QStringLiteral("未知");
    }
    }
}
}

bool parsePrimitiveEdit(const QString& input, const std::vector<Feature>& features,
    FeatureId selected, ModelRevision revision, PrimitiveEdit& output, QString& error)
{
    output = {};
    error.clear();
    auto fail = [&](const QString& message) { error = message; return false; };
    if (input.size() > 512) return fail(QStringLiteral("指令过长，请每次修改一个特征。"));
    auto text = input.trimmed();
    text.replace(QChar(0xFF03), QLatin1Char('#'));
    text.replace(QChar(0x2212), QLatin1Char('-'));
    if (text.startsWith(QStringLiteral("把")) || text.startsWith(QStringLiteral("将"))) text = text.mid(1).trimmed();
    FeatureId target = selected;
    const auto explicitId = QRegularExpression(QStringLiteral("^#([0-9]+)\\s*(?:的)?\\s*")).match(text);
    if (explicitId.hasMatch()) {
        bool ok = false;
        target = explicitId.captured(1).toULongLong(&ok);
        if (!ok || target == kInvalidFeatureId) return fail(QStringLiteral("特征编号无效。"));
        text.remove(0, explicitId.capturedLength());
    } else {
        const auto current = QRegularExpression(QStringLiteral("^(?:当前选中特征|当前特征|选中特征)\\s*(?:的)?\\s*")).match(text);
        if (current.hasMatch()) text.remove(0, current.capturedLength());
    }
    const Feature* feature = nullptr;
    for (const auto& candidate : features) if (candidate.id == target) { feature = &candidate; break; }
    if (!feature) return fail(QStringLiteral("请先选择特征，或使用存在的编号，例如“#3 的孔直径改为 8 mm”。"));
    if (feature->suppressed || !feature->valid) return fail(QStringLiteral("目标特征已抑制或无效，无法修改。"));
    PrimitiveEdit proposed;
    proposed.featureId = target;
    proposed.expectedRevision = revision;
    proposed.anchor = feature->resizeAnchor;
    QStringList labels;
    if (const auto* box = std::get_if<BoxPrimitiveParams>(&feature->params)) {
        proposed.values = {box->sizeX, box->sizeY, box->sizeZ};
        labels = {"长（局部 X）", "宽（局部 Y）", "高（局部 Z）"};
    } else if (const auto* cylinder = std::get_if<CylinderPrimitiveParams>(&feature->params)) {
        proposed.values = {cylinder->radius, cylinder->height, 1};
        labels = {"半径", "高度"};
    } else if (const auto* hole = std::get_if<ThroughHolePrimitiveParams>(&feature->params)) {
        proposed.values[0] = 2 * hole->radius;
        labels = {"孔直径"};
    } else return fail(QStringLiteral("当前支持长方体、圆柱和贯穿孔的尺寸指令。"));
    const auto before = proposed.values;
    auto clauses = text.split(QRegularExpression(QStringLiteral("[，,、]")), Qt::KeepEmptyParts);
    if (clauses.empty() || clauses.size() > 4) return fail(QStringLiteral("每次最多修改三个尺寸和一个固定参照。"));
    if (clauses.back().trimmed().startsWith(QStringLiteral("固定"))) {
        auto anchor = clauses.takeLast().trimmed().mid(2).simplified().remove(QLatin1Char(' ')).toUpper();
        if (anchor == QStringLiteral("中心")) proposed.anchor = ResizeAnchor::Center;
        else if (anchor == QStringLiteral("底面") || anchor == QStringLiteral("底面中心")) proposed.anchor = ResizeAnchor::Base;
        else if (anchor == QStringLiteral("顶面") || anchor == QStringLiteral("顶面中心")) proposed.anchor = ResizeAnchor::Top;
        else {
            const QStringList sides{"X-面", "X+面", "Y-面", "Y+面", "Z-面", "Z+面"};
            const int side = sides.indexOf(anchor);
            if (side < 0) return fail(QStringLiteral("固定参照可写“中心”“底面”“顶面”或“X-面”等局部轴面。"));
            proposed.anchor = static_cast<ResizeAnchor>(side + 1);
        }
    }
    if (clauses.empty()) return fail(QStringLiteral("请提供要修改的尺寸。"));
    const QRegularExpression clausePattern(QStringLiteral(
        "^(长(?:度)?|宽(?:度)?|高(?:度)?|[XYZxyz](?:方向)?(?:尺寸)?|半径|直径|孔径|孔直径)\\s*"
        "(改为|改成|设为|设置为|增加|增大|减少|减小)\\s*"
        "([+-]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)\\s*"
        "(毫米|厘米|米|mm|cm|m|%|％)?$"), QRegularExpression::CaseInsensitiveOption);
    std::set<int> changed;
    for (const auto& clause : clauses) {
        const auto match = clausePattern.match(clause.trimmed());
        if (!match.hasMatch()) return fail(QStringLiteral("无法完整理解指令。请使用“高度改为 30 mm”或“孔直径增加 2 mm”，多项用逗号分隔。"));
        const auto property = match.captured(1).toUpper();
        int index = -1;
        double factor = 1;
        if (std::holds_alternative<BoxPrimitiveParams>(feature->params)) {
            if (property.startsWith(QStringLiteral("长")) || property.startsWith('X')) index = 0;
            else if (property.startsWith(QStringLiteral("宽")) || property.startsWith('Y')) index = 1;
            else if (property.startsWith(QStringLiteral("高")) || property.startsWith('Z')) index = 2;
        } else if (std::holds_alternative<CylinderPrimitiveParams>(feature->params)) {
            if (property == QStringLiteral("半径")) index = 0;
            else if (property == QStringLiteral("直径")) { index = 0; factor = .5; }
            else if (property.startsWith(QStringLiteral("高"))) index = 1;
        } else if (property == QStringLiteral("直径") || property == QStringLiteral("孔径") || property == QStringLiteral("孔直径")) index = 0;
        if (index < 0) return fail(QStringLiteral("该特征不支持“%1”。可修改：%2。").arg(property, labels.join(QStringLiteral("、"))));
        if (!changed.insert(index).second) return fail(QStringLiteral("同一尺寸只能指定一次，半径和直径也不能重复指定。"));
        bool ok = false;
        double amount = match.captured(3).toDouble(&ok);
        if (!ok || !std::isfinite(amount)) return fail(QStringLiteral("尺寸必须是有限数字。"));
        const auto action = match.captured(2);
        const bool increase = action == QStringLiteral("增加") || action == QStringLiteral("增大");
        const bool decrease = action == QStringLiteral("减少") || action == QStringLiteral("减小");
        const auto unit = match.captured(4).toLower();
        if ((increase || decrease) && amount < 0) return fail(QStringLiteral("增加或减少的数值必须非负。"));
        if (unit == QStringLiteral("%") || unit == QStringLiteral("％")) {
            if (!increase && !decrease) return fail(QStringLiteral("百分比仅用于增加或减少；绝对尺寸请使用 mm、cm 或 m。"));
            proposed.values[index] *= 1 + (increase ? amount : -amount) / 100;
        } else {
            if (unit == QStringLiteral("cm") || unit == QStringLiteral("厘米")) amount *= 10;
            else if (unit == QStringLiteral("m") || unit == QStringLiteral("米")) amount *= 1000;
            amount *= factor;
            proposed.values[index] = increase ? before[index] + amount : decrease ? before[index] - amount : amount;
        }
    }
    FeatureParams adjusted;
    std::string failure;
    for (const int index : changed) {
        // Match the existing numeric editor's upper bound before invoking
        // native geometry on user-provided, potentially enormous numbers.
        if (!std::isfinite(proposed.values[index]) || proposed.values[index] > 1000000)
            return fail(QStringLiteral("修改后的尺寸必须为有限数字，且不大于 1000000 mm。"));
    }
    if (!adjustedPrimitive(feature->params, proposed.values, proposed.anchor, adjusted, failure))
        return fail(QStringLiteral("尺寸或固定参照无效：%1").arg(QString::fromStdString(failure)));
    proposed.summary = QStringLiteral("目标：%1 (#%2)\n").arg(QString::fromStdString(feature->name)).arg(target);
    for (const int index : changed)
        proposed.summary += QStringLiteral("%1：%2 → %3 mm\n").arg(labels[index], number(before[index]), number(proposed.values[index]));
    proposed.summary += QStringLiteral("固定参照：%1").arg(anchorName(proposed.anchor));
    output = std::move(proposed);
    return true;
}
}
