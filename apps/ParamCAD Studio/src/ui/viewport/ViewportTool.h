#pragma once

// The active in-viewport modeling tool. It lives in its own header so the tool
// palette that selects it and the viewer that consumes it do not have to depend
// on each other.
//
// Select is the SolidWorks arrow: a left click picks whatever is under the
// cursor. The other four place geometry on the active sketch plane.
enum class ViewportTool {
    Select,
    Point,
    Line,
    Rectangle,
    Circle,
};
