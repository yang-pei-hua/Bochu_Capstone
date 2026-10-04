#pragma once
#include <modeling/Feature.h>
#include <QJsonArray>
#include <QString>

namespace FeatureCodec {
QJsonArray encode(const std::vector<modeling::Feature>& features);
bool decode(const QJsonArray& json, std::vector<modeling::Feature>& features, QString& error);
} // namespace FeatureCodec
