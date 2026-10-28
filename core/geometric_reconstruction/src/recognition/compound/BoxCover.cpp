#include "detail/CompoundInternal.h"

namespace reconstruction::detail::compound {
bool selectBoxCover(const Arrangement& arrangement, const CompoundRecognitionOptions& options,
                    CompoundCandidate& output, std::string& error) {
    const auto& axes=arrangement.axes;
    const auto& boundaries=arrangement.boundaries;
    const auto& cells=arrangement.cells;
    const std::array<int,3> sizes{{int(boundaries[0].size()-1),int(boundaries[1].size()-1),int(boundaries[2].size()-1)}};
    const std::size_t total=std::size_t(sizes[0])*sizes[1]*sizes[2];
    auto index = [&](int x, int y, int z) { return (x * sizes[1] + y) * sizes[2] + z; };
    std::vector<Rectangle> rectangles;
    for (int x = 0; x < sizes[0]; ++x)
        for (int X = x + 1; X <= sizes[0]; ++X)
            for (int y = 0; y < sizes[1]; ++y)
                for (int Y = y + 1; Y <= sizes[1]; ++Y)
                    for (int z = 0; z < sizes[2]; ++z)
                        for (int Z = z + 1; Z <= sizes[2]; ++Z) {
                            Rectangle r;
                            r.lo = {{x, y, z}};
                            r.hi = {{X, Y, Z}};
                            bool valid = true;
                            for (int i = x; i < X && valid; ++i)
                                for (int j = y; j < Y && valid; ++j)
                                    for (int k = z; k < Z; ++k) {
                                        int id = index(i, j, k);
                                        if (!cells[id].full) {
                                            valid = false;
                                            break;
                                        }
                                        r.cells.push_back(id);
                                    }
                            if (valid) {
                                r.volume = (boundaries[0][X] - boundaries[0][x]) *
                                           (boundaries[1][Y] - boundaries[1][y]) *
                                           (boundaries[2][Z] - boundaries[2][z]);
                                rectangles.push_back(std::move(r));
                            }
                        }
    // Remove contained candidates. With overlap permitted, replacing a box by
    // a larger admissible box never increases the count or includes empty cells.
    std::vector<Rectangle> maximal;
    for (std::size_t i = 0; i < rectangles.size(); ++i) {
        bool contained = false;
        for (std::size_t j = 0; j < rectangles.size() && !contained; ++j)
            if (i != j && rectangles[j].cells.size() > rectangles[i].cells.size()) {
                contained = true;
                for (int a = 0; a < 3; ++a)
                    contained &= rectangles[j].lo[a] <= rectangles[i].lo[a] &&
                                 rectangles[j].hi[a] >= rectangles[i].hi[a];
            }
        if (!contained)
            maximal.push_back(rectangles[i]);
    }
    rectangles = std::move(maximal);
    std::sort(rectangles.begin(), rectangles.end(),
              [](const auto &a, const auto &b) { return a.cells.size() > b.cells.size(); });
    std::vector<std::vector<int>> containing(total);
    for (std::size_t i = 0; i < rectangles.size(); ++i)
        for (int c : rectangles[i].cells)
            containing[c].push_back(int(i));
    std::vector<bool> covered(total, false);
    std::vector<int> chosen, bestCover;
    std::size_t fullCount = 0;
    for (const auto &c : cells)
        fullCount += c.full;
    // A greedy cover bounds the exact branch-and-bound search immediately.
    std::size_t remaining = fullCount;
    while (remaining) {
        int winner = -1;
        std::size_t gain = 0;
        for (std::size_t i = 0; i < rectangles.size(); ++i) {
            std::size_t g = 0;
            for (int c : rectangles[i].cells)
                g += !covered[c];
            if (g > gain) {
                gain = g;
                winner = int(i);
            }
        }
        if (winner < 0) {
            error = "Material cell has no admissible box";
            return false;
        }
        bestCover.push_back(winner);
        for (int c : rectangles[winner].cells)
            if (!covered[c]) {
                covered[c] = true;
                --remaining;
            }
    }
    std::fill(covered.begin(), covered.end(), false);
    bool exhausted = false;
    std::function<void(std::size_t)> search = [&](std::size_t left) {
        if (++output.searchNodes > options.maximumSearchNodes) {
            exhausted = true;
            return;
        }
        if (!left) {
            if (chosen.size() < bestCover.size())
                bestCover = chosen;
            return;
        }
        if (chosen.size() + 1 >= bestCover.size())
            return;
        int pivot = -1;
        std::size_t alternatives = std::numeric_limits<std::size_t>::max();
        for (std::size_t c = 0; c < total; ++c)
            if (cells[c].full && !covered[c] && containing[c].size() < alternatives) {
                pivot = int(c);
                alternatives = containing[c].size();
            }
        if (pivot < 0)
            return;
        for (int r : containing[pivot]) {
            std::vector<int> changed;
            for (int c : rectangles[r].cells)
                if (!covered[c]) {
                    covered[c] = true;
                    changed.push_back(c);
                }
            chosen.push_back(r);
            search(left - changed.size());
            chosen.pop_back();
            for (int c : changed)
                covered[c] = false;
            if (exhausted)
                return;
        }
    };
    search(fullCount);
    output.minimumBoxCountProven = !exhausted;
    for (int r : bestCover) {
        const auto &rect = rectangles[r];
        Coord origin{};
        BoxCandidate box;
        for (int a = 0; a < 3; ++a)
            origin[a] = boundaries[a][rect.lo[a]];
        box.primitive.pose = {world(origin, axes), axes[0], axes[2]};
        box.primitive.sizeX = boundaries[0][rect.hi[0]] - origin[0];
        box.primitive.sizeY = boundaries[1][rect.hi[1]] - origin[1];
        box.primitive.sizeZ = boundaries[2][rect.hi[2]] - origin[2];
        box.confidence = 1 - output.occupancyError;
        output.boxes.push_back(box);
    }
    if (output.boxes.empty() && output.prisms.empty()) {
        error = "No supported material";
        return false;
    }
    return true;
}
} // namespace reconstruction::detail::compound
