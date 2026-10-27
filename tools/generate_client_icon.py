"""Regenerate the client's SVG, PNG and Windows ICO assets (requires Pillow)."""

from pathlib import Path
from xml.sax.saxutils import escape

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
DESTINATION = ROOT / "apps" / "ParamCAD Studio" / "resources" / "icons"
SCALE = 4
# Fill the desktop icon tile while keeping a small transparent safety margin.
MARK_SCALE = 1.44


def canvas_point(x, y):
    return (256 + (x - 260) * MARK_SCALE) * SCALE, (256 + (y - 257) * MARK_SCALE) * SCALE


def main():
    DESTINATION.mkdir(parents=True, exist_ok=True)
    image = Image.new("RGBA", (512 * SCALE, 512 * SCALE))
    draw = ImageDraw.Draw(image)
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="512" height="512" viewBox="0 0 512 512">',
        '<title>ParamCAD Studio</title>',
        '<desc>A chamfered steel tooling block with a machined bore and B and P engraved into its two side faces, for Bochu Parameter Studio.</desc>',
        f'<g transform="translate(256 256) scale({MARK_SCALE}) translate(-260 -257)">',
    ]

    def polygon(points, color):
        draw.polygon([canvas_point(x, y) for x, y in points], fill=color)
        coordinates = " ".join(f"{x},{y}" for x, y in points)
        svg.append(f'<polygon points="{coordinates}" fill="{escape(color)}"/>')

    def line(points, color, width):
        scaled = [canvas_point(x, y) for x, y in points]
        draw.line(scaled, fill=color, width=round(width * MARK_SCALE * SCALE), joint="curve")
        coordinates = " ".join(f"{x},{y}" for x, y in points)
        svg.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="butt" stroke-linejoin="miter"/>')

    def ellipse(x, y, rx, ry, color):
        draw.ellipse((*canvas_point(x - rx, y - ry),
                      *canvas_point(x + rx, y + ry)), fill=color)
        svg.append(f'<ellipse cx="{x}" cy="{y}" rx="{rx}" ry="{ry}" fill="{color}"/>')

    # Orthographic isometric projection of a chamfered machining blank.
    top = [(274, 94), (414, 174), (414, 190), (274, 270),
           (246, 270), (106, 190), (106, 174), (246, 94)]
    depth = 150
    bottom = [(x, y + depth) for x, y in top]
    for index, color in ((1, "#71818C"), (2, "#465968"),
                         (3, "#657783"), (4, "#88959E"), (5, "#ACB6BD")):
        following = index + 1
        polygon([top[index], top[following], bottom[following], bottom[index]], color)
        line([top[index], bottom[index], bottom[following]], "#2B3A45", 3)
    line([top[6], bottom[6], bottom[5]], "#D6DFE4", 3)
    polygon(top, "#9CAAB5")
    inset = [(260 + (x - 260) * 0.945, 182 + (y - 182) * 0.945)
             for x, y in top]
    polygon(inset, "#D4DBDF")
    line(top + [top[0]], "#E8EDF0", 3)
    # Countersunk rim, dark bore and the visible cylindrical inner wall.
    ellipse(260, 182, 79, 45, "#EFF3F5")
    ellipse(260, 184, 74, 42, "#718593")
    ellipse(260, 185, 64, 36, "#182832")
    ellipse(260, 197, 52, 20, "#415765")
    ellipse(260, 199, 48, 15, "#273D4A")
    # Custom angular letter outlines keep the assets independent of local fonts.
    # Both glyphs follow the same isometric basis as their respective side face.
    b_contours = [
        [(0, 0), (57, 0), (78, 18), (78, 44), (66, 56),
         (82, 70), (82, 94), (63, 112), (0, 112)],
        [(22, 19), (49, 19), (57, 27), (57, 38), (49, 45), (22, 45)],
        [(22, 66), (52, 66), (61, 74), (61, 86), (52, 94), (22, 94)],
    ]
    p_contours = [
        [(0, 0), (58, 0), (80, 20), (80, 53), (59, 73),
         (22, 73), (22, 112), (0, 112)],
        [(22, 20), (49, 20), (58, 29), (58, 43), (49, 53), (22, 53)],
    ]

    def engrave(contours, origin, slope, face, recess, highlight):
        for offset, color in ((2, highlight), (0, recess)):
            for index, contour in enumerate(contours):
                projected = [(origin[0] + 0.95 * x + offset,
                              origin[1] + slope * x + 0.95 * y + offset)
                             for x, y in contour]
                polygon(projected, face if index else color)

    engrave(b_contours, (136, 236), 0.95 * 80 / 140,
            "#88959E", "#43535F", "#BCC6CD")
    engrave(p_contours, (302, 284), -0.95 * 80 / 140,
            "#465968", "#20343F", "#8297A5")
    svg.extend(["</g>", "</svg>"])
    (DESTINATION / "paramcad-studio.svg").write_text("\n".join(svg) + "\n", encoding="utf-8")
    image = image.resize((512, 512), Image.Resampling.LANCZOS)
    image.save(DESTINATION / "paramcad-studio.png")
    image.save(DESTINATION / "paramcad-studio.ico",
               sizes=[(size, size) for size in (16, 20, 24, 32, 40, 48, 64, 96, 128, 256)])
    print(f"Generated SVG, PNG and ICO in {DESTINATION}")


if __name__ == "__main__":
    main()
