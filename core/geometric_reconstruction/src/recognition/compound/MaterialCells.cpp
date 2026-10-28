#include "detail/MaterialQuery.h"

namespace reconstruction::detail::compound {
bool classifyMaterialCells(const PointStore& store, const CompoundRecognitionOptions& options,
                           Arrangement& arrangement, CompoundCandidate& output, std::string& error) {
    const auto& points=store.points();
    const auto& axes=arrangement.axes;
    const auto& boundaries=arrangement.boundaries;
    const auto& obliques=arrangement.obliques;
    std::array<int, 3> sizes{{int(boundaries[0].size() - 1), int(boundaries[1].size() - 1),
                              int(boundaries[2].size() - 1)}};
    const std::size_t total = std::size_t(sizes[0]) * sizes[1] * sizes[2];
    if (total > options.maximumGridCells) {
        error = "Planar arrangement exceeds grid limit";
        return false;
    }
    output.gridCells = total;
    Nearest nearest(points, options.distanceTolerance);
    auto& cells=arrangement.cells;
    double totalError = 0;
    for (int x = 0; x < sizes[0]; ++x)
        for (int y = 0; y < sizes[1]; ++y)
            for (int z = 0; z < sizes[2]; ++z) {
                Coord lo{{boundaries[0][x], boundaries[1][y], boundaries[2][z]}},
                    hi{{boundaries[0][x + 1], boundaries[1][y + 1], boundaries[2][z + 1]}};
                std::vector<std::pair<Coord, bool>> samples;
                int insideCount = 0;
                for (int i = 0; i < 5; ++i)
                    for (int j = 0; j < 5; ++j)
                        for (int k = 0; k < 5; ++k) {
                            Coord q{{lo[0] + (hi[0] - lo[0]) * (i + 0.5) / 5,
                                     lo[1] + (hi[1] - lo[1]) * (j + 0.5) / 5,
                                     lo[2] + (hi[2] - lo[2]) * (k + 0.5) / 5}};
                            const bool inside = nearest.inside(world(q, axes));
                            insideCount += inside;
                            samples.push_back({q, inside});
                        }
                const double ratio = insideCount / 125.0;
                if (ratio >= 1 - options.maximumCellError) {
                    cells.push_back({{{x, y, z}}, true});
                    totalError += 1 - ratio;
                    continue;
                }
                if (ratio <= options.maximumCellError) {
                    cells.push_back({{{x, y, z}}, false});
                    totalError += ratio;
                    continue;
                }
                double bestError = 1;
                const Oblique *best = nullptr;
                double bestSign = 1;
                for (const auto &plane : obliques)
                    for (double sign : {-1.0, 1.0}) {
                        int mismatch = 0;
                        for (const auto &sample : samples) {
                            double d = plane.offset;
                            for (int a = 0; a < 3; ++a)
                                d += plane.n[a] * sample.first[a];
                            mismatch += (sign * d <= 0) != sample.second;
                        }
                        if (mismatch / 125.0 < bestError) {
                            bestError = mismatch / 125.0;
                            best = &plane;
                            bestSign = sign;
                        }
                    }
                if (!best || bestError > options.maximumCellError) {
                    error = "Unexplained material cell " + std::to_string(x) + "," +
                            std::to_string(y) + "," + std::to_string(z) +
                            "; occupancy=" + std::to_string(ratio) +
                            "; best prism error=" + std::to_string(bestError);
                    return false;
                }
                totalError += bestError;
                cells.push_back({{{x, y, z}}, false});
                const int e = best->extrusion, u = (e + 1) % 3, v = (e + 2) % 3;
                auto polygon = clipPolygon(
                    {{lo[u], lo[v]}, {hi[u], lo[v]}, {hi[u], hi[v]}, {lo[u], hi[v]}},
                    bestSign * best->n[u], bestSign * best->n[v], bestSign * best->offset);
                if (polygon.size() < 3) {
                    error = "Degenerate prism intersection";
                    return false;
                }
                // cyclic axes ensure normal cross xDirection equals the polygon Y axis.
                Coord origin{};
                origin[e] = lo[e];
                PrismCandidate prism;
                prism.plane = {world(origin, axes), axes[e], axes[u]};
                prism.depth = hi[e] - lo[e];
                prism.polygon = std::move(polygon);
                output.prisms.push_back(std::move(prism));
            }
    output.occupancyError = totalError / total;
    return true;
}
} // namespace reconstruction::detail::compound
