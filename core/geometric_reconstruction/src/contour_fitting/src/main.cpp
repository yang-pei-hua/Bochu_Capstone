#include "contour_fitting/ContourFitting.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::vector<contour_fitting::Point2D> readPoints(const char* path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open the input CSV");
    std::vector<contour_fitting::Point2D> points;
    std::string line;
    bool first = true;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        if (++lineNumber == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
        const auto begin = line.find_first_not_of(" \t\r");
        if (begin == std::string::npos) continue;
        const auto end = line.find_last_not_of(" \t\r");
        if (first && line.substr(begin, end - begin + 1) == "x,y") {
            first = false;
            continue;
        }
        first = false;
        std::istringstream row(line);
        row.imbue(std::locale::classic());
        contour_fitting::Point2D point;
        char comma = 0;
        if (!(row >> point.x >> comma >> point.y) || comma != ',' || !(row >> std::ws).eof())
            throw std::runtime_error("Expected x,y on CSV line " + std::to_string(lineNumber));
        points.push_back(point);
    }
    if (!input.eof()) throw std::runtime_error("Cannot finish reading the input CSV");
    return points;
}

double readNumber(const char* value)
{
    std::istringstream input(value);
    input.imbue(std::locale::classic());
    double number = 0;
    if (!(input >> number) || !(input >> std::ws).eof())
        throw std::invalid_argument("Expected a numeric fitting option");
    return number;
}

void writeRing(std::ostream& output, const contour_fitting::Ring& ring)
{
    output << '[';
    for (std::size_t i = 0; i < ring.size(); ++i) {
        if (i) output << ',';
        output << '[' << ring[i].x << ',' << ring[i].y << ']';
    }
    output << ']';
}

void writeResult(const char* path, const contour_fitting::Result& result)
{
    std::ofstream output(path);
    if (!output) throw std::runtime_error("Cannot open the output JSON");
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "{\n  \"unique_points\": " << result.uniquePointCount
           << ",\n  \"alpha_radius\": " << result.alphaRadius
           << ",\n  \"simplification_tolerance\": " << result.simplificationTolerance
           << ",\n  \"contours\": [";
    for (std::size_t i = 0; i < result.contours.size(); ++i) {
        if (i) output << ',';
        const auto& contour = result.contours[i];
        output << "\n    {\"outer\": ";
        writeRing(output, contour.outer);
        output << ", \"holes\": [";
        for (std::size_t j = 0; j < contour.holes.size(); ++j) {
            if (j) output << ',';
            writeRing(output, contour.holes[j]);
        }
        output << "]}";
    }
    output << "\n  ]\n}\n";
    output.close();
    if (!output) throw std::runtime_error("Cannot finish writing the output JSON");
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 5) {
        std::cerr << "Usage: fit_contour input.csv output.json [alpha_radius] [simplification_tolerance]\n";
        return 1;
    }
    try {
        contour_fitting::Options options;
        if (argc >= 4) options.alphaRadius = readNumber(argv[3]);
        if (argc == 5) options.simplificationTolerance = readNumber(argv[4]);
        const auto result = contour_fitting::fitContours(readPoints(argv[1]), options);
        writeResult(argv[2], result);
        if (result.contours.empty()) {
            std::cerr << "No closed contour at the requested scale\n";
            return 2;
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
