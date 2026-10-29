#include "ui/panels/FeatureEditorPanel.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <algorithm>

using modeling::ResizeAnchor;

FeatureEditorPanel::FeatureEditorPanel(QWidget* parent) : QWidget(parent)
{
    setObjectName("featureEditorPanel");
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    auto* group = new QGroupBox(tr("参数调整（mm）"), this);
    auto* form = new QFormLayout(group);
    m_target=new QComboBox(group);m_target->setObjectName("profileBodyEditTarget");m_targetLabel=new QLabel(tr("调整对象"),group);
    form->addRow(m_targetLabel,m_target);
    m_anchor = new QComboBox(group); m_anchor->setObjectName("featureResizeAnchor");
    form->addRow(tr("保持固定"), m_anchor);
    for (size_t i = 0; i < 3; ++i) {
        m_labels[i] = new QLabel(group);
        m_values[i] = new QDoubleSpinBox(group);
        m_values[i]->setObjectName(QStringLiteral("featureDimension%1").arg(i));
        m_values[i]->setRange(0.000001,1000000);
        m_values[i]->setDecimals(6); m_values[i]->setSingleStep(1);
        m_values[i]->setSuffix(" mm"); m_values[i]->setKeyboardTracking(false);
        form->addRow(m_labels[i],m_values[i]);
    }
    m_choose = new QPushButton(tr("在视口选择参照面 / 中心"), group);
    m_choose->setObjectName("chooseFeatureReferenceButton");
    form->addRow(m_choose);
    auto* buttons = new QHBoxLayout;
    m_apply = new QPushButton(tr("应用"), group); m_apply->setObjectName("applyFeatureParametersButton");
    auto* cancel = new QPushButton(tr("取消"), group); cancel->setObjectName("cancelFeatureParametersButton");
    buttons->addWidget(m_apply); buttons->addWidget(cancel);
    form->addRow(buttons);
    m_message = new QLabel(group); m_message->setObjectName("featureEditMessage");
    m_message->setWordWrap(true); form->addRow(m_message);
    layout->addWidget(group);
    connect(m_anchor, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
        resetFields(); updateControls();
    });
    connect(m_target,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{
        // A restored/surface-only body may no longer have the former cavity
        // target. An empty selection must resolve to the overall dimensions.
        if(m_target->currentIndex()<0 && m_target->count()>0){const QSignalBlocker blocker(m_target);m_target->setCurrentIndex(0);}
        resetFields();updateControls();});
    connect(m_choose,&QPushButton::clicked,this,&FeatureEditorPanel::selectReferenceRequested);
    connect(cancel,&QPushButton::clicked,this,[this] {
        if (m_feature) setAnchor(m_feature->resizeAnchor);
        resetFields(); updateControls(); emit cancelRequested();
    });
    connect(m_apply,&QPushButton::clicked,this,[this] {
        for (auto* value : m_values) value->interpretText();
        if (!m_feature || !m_allowed) return;
        auto values=m_committedValues;
        for (size_t i=0;i<3;++i)
            if (m_values[i]->isEnabled() && m_values[i]->value()!=m_displayedValues[i]) values[i]=m_values[i]->value();
        if(m_feature->type==modeling::FeatureType::ProfileBody && m_target->currentData().toInt()!=-1) {
            emit bodyCavityApplyRequested(m_feature->id,m_target->currentData().toInt(),values[0],values[1],values[2],m_revision);return;
        }
        emit applyRequested(m_feature->id,values[0],values[1],values[2],
            static_cast<ResizeAnchor>(m_anchor->currentData().toInt()),m_revision);
    });
    setFeature(nullptr,0);
}

modeling::FeatureId FeatureEditorPanel::featureId() const
{
    return m_feature ? m_feature->id : modeling::kInvalidFeatureId;
}

void FeatureEditorPanel::setFeature(const modeling::Feature* feature, modeling::ModelRevision revision)
{
    const bool supported = feature && (feature->type == modeling::FeatureType::BoxPrimitive
        || feature->type == modeling::FeatureType::ThroughHolePrimitive
        || feature->type == modeling::FeatureType::CylinderPrimitive || feature->type==modeling::FeatureType::ProfileBody);
    const int previousTarget=m_feature && feature && m_feature->id==feature->id?m_target->currentData().toInt():-1;
    m_feature = supported ? std::optional<modeling::Feature>(*feature) : std::nullopt;
    m_revision = revision; setVisible(supported);
    if (!supported) return;
    const bool profile=feature->type==modeling::FeatureType::ProfileBody;
    m_target->setVisible(profile);m_targetLabel->setVisible(profile);
    {const QSignalBlocker targetBlocker(m_target);m_target->clear();
        if(profile){m_target->addItem(tr("整体缩放尺寸"),-1);const auto& p=std::get<modeling::ProfileBodyParams>(feature->params);
            for(std::size_t i=0;i<p.envelope.size();++i)m_target->addItem(tr("外形回转段 %1").arg(i+1),-int(i)-2);
            for(std::size_t i=0;i<p.cavities.size();++i) {
                m_target->addItem(tr("孔 / 腔段 %1：外径").arg(i+1),int(i*2));
                if(std::max(p.cavities[i].innerStartRadius,p.cavities[i].innerEndRadius)>0)
                    m_target->addItem(tr("环形腔段 %1：内径").arg(i+1),int(i*2+1));
            }
            m_target->setCurrentIndex(std::max(0,m_target->findData(previousTarget)));}}
    const QSignalBlocker blocker(m_anchor);
    m_anchor->clear();
    const auto add = [this](const QString& text, ResizeAnchor anchor) {
        m_anchor->addItem(text, static_cast<int>(anchor));
    };
    if (feature->type == modeling::FeatureType::BoxPrimitive) {
        add(tr("中心：三个方向对称调整"),ResizeAnchor::Center);
        const QStringList sides{"X− 面","X+ 面","Y− 面","Y+ 面","Z− 面","Z+ 面"};
        for (int i=0;i<6;++i) add(sides[i],static_cast<ResizeAnchor>(i+1));
        m_labels[0]->setText(tr("长 / 局部 X")); m_labels[1]->setText(tr("宽 / 局部 Y"));
        m_labels[2]->setText(tr("高 / 局部 Z"));
    } else if(profile) {
        add(tr("零件中心 / 孔段起始位置"),ResizeAnchor::Center);
    } else if (feature->type == modeling::FeatureType::CylinderPrimitive) {
        add(tr("底面中心"),ResizeAnchor::Base); add(tr("顶面中心"),ResizeAnchor::Top);
        add(tr("实体中心"),ResizeAnchor::Center);
        m_labels[0]->setText(tr("半径")); m_labels[1]->setText(tr("高度"));
    } else { add(tr("孔轴的位置和方向"),ResizeAnchor::Center); m_labels[0]->setText(tr("孔直径")); }
    m_anchor->setCurrentIndex(m_anchor->findData(static_cast<int>(feature->resizeAnchor)));
    resetFields(); updateControls();
}

void FeatureEditorPanel::setAnchor(ResizeAnchor anchor)
{
    m_anchor->setCurrentIndex(m_anchor->findData(static_cast<int>(anchor)));
}

void FeatureEditorPanel::resetFields()
{
    if (!m_feature) return;
    std::array<double,3> values{1,1,1};
    if (const auto* box = std::get_if<modeling::BoxPrimitiveParams>(&m_feature->params))
        values={box->sizeX,box->sizeY,box->sizeZ};
    else if (const auto* cylinder = std::get_if<modeling::CylinderPrimitiveParams>(&m_feature->params))
        values={cylinder->radius,cylinder->height,1};
    else if (const auto* hole = std::get_if<modeling::ThroughHolePrimitiveParams>(&m_feature->params)) values[0]=hole->radius*2;
    const auto* body=std::get_if<modeling::ProfileBodyParams>(&m_feature->params);
    const bool cavity=body && m_target->currentData().toInt()!=-1;
    if(body) {
        if(cavity){const int target=m_target->currentData().toInt();const auto& c=target<=-2?body->envelope.at(std::size_t(-target-2)):body->cavities.at(std::size_t(target/2));
            values={2*(target>=0 && target%2?c.innerStartRadius:c.startRadius),2*(target>=0 && target%2?c.innerEndRadius:c.endRadius),c.end-c.start};
            m_labels[0]->setText(tr("基准起始直径"));m_labels[1]->setText(tr("基准终止直径"));m_labels[2]->setText(tr("基准段深度"));}
        else {values={body->sizeX,body->sizeY,body->sizeZ};m_labels[0]->setText(tr("整体局部 X"));m_labels[1]->setText(tr("整体局部 Y"));m_labels[2]->setText(tr("整体局部 Z"));}
    }
    m_committedValues=values;
    for (size_t i=0;i<3;++i) {
        m_values[i]->setMinimum(cavity && i<2?0:0.000001);
        m_values[i]->setValue(values[i]); m_displayedValues[i]=m_values[i]->value();
    }
}

void FeatureEditorPanel::setEditingAllowed(bool allowed) { m_allowed=allowed; updateControls(); }
void FeatureEditorPanel::setMessage(const QString& message) { m_message->setText(message); }

void FeatureEditorPanel::updateControls()
{
    if (!m_feature) return;
    const bool box = m_feature->type == modeling::FeatureType::BoxPrimitive;
    const int count = box || m_feature->type==modeling::FeatureType::ProfileBody ? 3 : m_feature->type == modeling::FeatureType::CylinderPrimitive ? 2 : 1;
    const int anchor = m_anchor->currentData().toInt();
    const bool enabled = m_allowed && !m_feature->suppressed;
    for (int i=0;i<3;++i) {
        m_labels[i]->setVisible(i<count); m_values[i]->setVisible(i<count);
        m_values[i]->setEnabled(enabled && (!box || anchor==0 || (anchor-1)/2==i));
    }
    m_anchor->setEnabled(enabled); m_apply->setEnabled(enabled);
    m_target->setEnabled(enabled);
    m_choose->setVisible(box); m_choose->setEnabled(enabled);
    if (!enabled) setMessage(tr("当前显示不是该参数模型，或特征已抑制；无法修改。"));
    else if (box) setMessage(tr("固定面仅调整其法向尺寸。其他特征的位置保持原建模坐标；应用后可撤销。"));
    else setMessage(tr("输入为待提交尺寸。应用后可撤销；手动修改会受到重拟合保护。"));
}
