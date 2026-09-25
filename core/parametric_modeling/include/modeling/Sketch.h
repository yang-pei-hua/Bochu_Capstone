#pragma once

#include "modeling/Id.h"

#include <variant>

namespace modeling {

struct Line2D {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

struct Rectangle2D {
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
};

struct Circle2D {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
};

enum class SketchEntityType {
    Line,
    Rectangle,
    Circle,
};

using SketchGeometry = std::variant<Line2D, Rectangle2D, Circle2D>;

struct SketchEntity {
    SketchEntityId id = kInvalidSketchEntityId;
    SketchGeometry geometry;
};

inline SketchEntityType sketchEntityType(const SketchEntity& entity) {
    if (std::holds_alternative<Line2D>(entity.geometry)) {
        return SketchEntityType::Line;
    }
    if (std::holds_alternative<Rectangle2D>(entity.geometry)) {
        return SketchEntityType::Rectangle;
    }
    return SketchEntityType::Circle;
}

}  // namespace modeling
