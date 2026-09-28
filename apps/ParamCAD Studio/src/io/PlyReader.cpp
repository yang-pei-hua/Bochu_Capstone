#include "io/PlyReader.h"

#include <QByteArray>
#include <QFile>
#include <QLatin1String>
#include <QRegularExpression>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {

enum class Format
{
    Ascii,
    BinaryLittleEndian,
    BinaryBigEndian,
};

// One PLY scalar type: its byte width plus how to interpret the bits.
struct ScalarType
{
    int size = 0;
    char code = 0; // 'i' signed integer, 'u' unsigned integer, 'f' floating point
};

struct Property
{
    QString name;
    ScalarType scalar;
    bool isList = false;
    ScalarType countType;
    ScalarType itemType;
};

struct Element
{
    QString name;
    quint64 count = 0;
    std::vector<Property> properties;
};

bool lookupScalarType(const QString& type, ScalarType& result)
{
    struct Entry
    {
        const char* name;
        int size;
        char code;
    };
    static const Entry entries[] = {
        {"char", 1, 'i'},   {"int8", 1, 'i'},
        {"uchar", 1, 'u'},  {"uint8", 1, 'u'},
        {"short", 2, 'i'},  {"int16", 2, 'i'},
        {"ushort", 2, 'u'}, {"uint16", 2, 'u'},
        {"int", 4, 'i'},    {"int32", 4, 'i'},
        {"uint", 4, 'u'},   {"uint32", 4, 'u'},
        {"float", 4, 'f'},  {"float32", 4, 'f'},
        {"double", 8, 'f'}, {"float64", 8, 'f'},
    };
    for (const Entry& entry : entries) {
        if (type == QLatin1String(entry.name)) {
            result.size = entry.size;
            result.code = entry.code;
            return true;
        }
    }
    return false;
}

QStringList splitFields(const QString& line)
{
    static const QRegularExpression separator(QStringLiteral("\\s+"));
    return line.split(separator, Qt::SkipEmptyParts);
}

// Reassembles the bytes into an integer and reinterprets them, so the result is
// independent of the host byte order.
bool decodeBinaryScalar(const char* data, const ScalarType& type, bool littleEndian, double& result)
{
    quint64 raw = 0;
    if (littleEndian) {
        for (int index = type.size - 1; index >= 0; --index) {
            raw = (raw << 8) | static_cast<unsigned char>(data[index]);
        }
    } else {
        for (int index = 0; index < type.size; ++index) {
            raw = (raw << 8) | static_cast<unsigned char>(data[index]);
        }
    }

    switch (type.code) {
    case 'u':
        result = static_cast<double>(raw);
        return true;
    case 'i': {
        const int bits = type.size * 8;
        const qint64 value = static_cast<qint64>(raw << (64 - bits)) >> (64 - bits);
        result = static_cast<double>(value);
        return true;
    }
    case 'f':
        if (type.size == 4) {
            const quint32 bits = static_cast<quint32>(raw);
            float value = 0.0F;
            std::memcpy(&value, &bits, sizeof(value));
            result = static_cast<double>(value);
            return true;
        }
        if (type.size == 8) {
            double value = 0.0;
            std::memcpy(&value, &raw, sizeof(value));
            result = value;
            return true;
        }
        break;
    default:
        break;
    }
    return false;
}

bool parseHeader(QFile& file, Format& format, std::vector<Element>& elements, QString& error)
{
    bool sawFormat = false;
    while (true) {
        const QByteArray rawLine = file.readLine();
        if (rawLine.isEmpty()) {
            error = QStringLiteral("PLY header ended before end_header");
            return false;
        }
        const QStringList parts = splitFields(QString::fromLatin1(rawLine).trimmed());
        if (parts.isEmpty()) {
            continue;
        }

        const QString& keyword = parts.first();
        if (keyword == QLatin1String("comment") || keyword == QLatin1String("obj_info")) {
            continue;
        }
        if (keyword == QLatin1String("format")) {
            if (parts.size() < 2) {
                error = QStringLiteral("PLY format line is incomplete");
                return false;
            }
            const QString& name = parts.at(1);
            if (name == QLatin1String("ascii")) {
                format = Format::Ascii;
            } else if (name == QLatin1String("binary_little_endian")) {
                format = Format::BinaryLittleEndian;
            } else if (name == QLatin1String("binary_big_endian")) {
                format = Format::BinaryBigEndian;
            } else {
                error = QStringLiteral("unsupported PLY format: %1").arg(name);
                return false;
            }
            sawFormat = true;
            continue;
        }
        if (keyword == QLatin1String("element")) {
            if (parts.size() < 3) {
                error = QStringLiteral("PLY element line is incomplete");
                return false;
            }
            Element element;
            element.name = parts.at(1);
            bool ok = false;
            element.count = parts.at(2).toULongLong(&ok);
            if (!ok) {
                error = QStringLiteral("PLY element %1 has an invalid count").arg(element.name);
                return false;
            }
            elements.push_back(element);
            continue;
        }
        if (keyword == QLatin1String("property")) {
            if (elements.empty()) {
                error = QStringLiteral("PLY property declared before any element");
                return false;
            }
            Property property;
            if (parts.size() >= 2 && parts.at(1) == QLatin1String("list")) {
                if (parts.size() < 5) {
                    error = QStringLiteral("PLY list property line is incomplete");
                    return false;
                }
                property.isList = true;
                if (!lookupScalarType(parts.at(2), property.countType)
                    || !lookupScalarType(parts.at(3), property.itemType)) {
                    error = QStringLiteral("unsupported PLY list property types: %1 %2")
                                .arg(parts.at(2), parts.at(3));
                    return false;
                }
                property.name = parts.at(4);
            } else {
                if (parts.size() < 3) {
                    error = QStringLiteral("PLY property line is incomplete");
                    return false;
                }
                if (!lookupScalarType(parts.at(1), property.scalar)) {
                    error = QStringLiteral("unsupported PLY property type: %1").arg(parts.at(1));
                    return false;
                }
                property.name = parts.at(2);
            }
            elements.back().properties.push_back(property);
            continue;
        }
        if (keyword == QLatin1String("end_header")) {
            break;
        }
        // Plenty of writers put vendor data in the header; ignore what we do
        // not understand rather than rejecting the whole file.
    }

    if (!sawFormat) {
        error = QStringLiteral("PLY header has no format line");
        return false;
    }
    return true;
}

// Consumes bytes without materialising them, so a huge face element does not
// cost extra memory.
bool advance(QFile& file, quint64 bytes)
{
    char scratch[64];
    while (bytes > 0) {
        const qint64 chunk = static_cast<qint64>(std::min<quint64>(bytes, sizeof(scratch)));
        const qint64 read = file.read(scratch, chunk);
        if (read < chunk) {
            return false;
        }
        bytes -= static_cast<quint64>(read);
    }
    return true;
}

bool readFully(QFile& file, quint64 bytes, QByteArray& out)
{
    if (bytes > static_cast<quint64>(std::numeric_limits<int>::max())) {
        return false;
    }
    out.resize(static_cast<int>(bytes));
    qint64 total = 0;
    while (total < static_cast<qint64>(bytes)) {
        const qint64 read = file.read(out.data() + total, static_cast<qint64>(bytes) - total);
        if (read <= 0) {
            return false;
        }
        total += read;
    }
    return true;
}

unsigned char toColorByte(double value)
{
    return static_cast<unsigned char>(std::lround(std::clamp(value, 0.0, 255.0)));
}

bool readVertices(QFile& file, const Element& element, Format format, PlyCloud& cloud, QString& error)
{
    for (const Property& property : element.properties) {
        if (property.isList) {
            error = QStringLiteral("PLY vertex element uses an unsupported list property: %1")
                        .arg(property.name);
            return false;
        }
    }

    std::vector<int> offsets;
    offsets.reserve(element.properties.size());
    int stride = 0;
    int xIndex = -1;
    int yIndex = -1;
    int zIndex = -1;
    int redIndex = -1;
    int greenIndex = -1;
    int blueIndex = -1;
    for (std::size_t index = 0; index < element.properties.size(); ++index) {
        const Property& property = element.properties.at(index);
        offsets.push_back(stride);
        stride += property.scalar.size;
        const int slot = static_cast<int>(index);
        if (property.name == QLatin1String("x")) {
            xIndex = slot;
        } else if (property.name == QLatin1String("y")) {
            yIndex = slot;
        } else if (property.name == QLatin1String("z")) {
            zIndex = slot;
        } else if (property.name == QLatin1String("red") || property.name == QLatin1String("r")) {
            redIndex = slot;
        } else if (property.name == QLatin1String("green") || property.name == QLatin1String("g")) {
            greenIndex = slot;
        } else if (property.name == QLatin1String("blue") || property.name == QLatin1String("b")) {
            blueIndex = slot;
        }
    }

    if (stride <= 0 || xIndex < 0 || yIndex < 0 || zIndex < 0) {
        error = QStringLiteral("PLY vertex element has no x/y/z properties");
        return false;
    }
    // Colors are all-or-nothing: a partial set would desynchronise the two
    // arrays, and a cloud without colors still renders fine.
    const bool hasColor = redIndex >= 0 && greenIndex >= 0 && blueIndex >= 0;

    const quint64 count = element.count;
    cloud.positions.clear();
    cloud.colors.clear();
    cloud.positions.reserve(static_cast<std::size_t>(count));
    if (hasColor) {
        cloud.colors.reserve(static_cast<std::size_t>(count));
    }

    if (format == Format::Ascii) {
        const int expected = static_cast<int>(element.properties.size());
        for (quint64 index = 0; index < count; ++index) {
            const QByteArray rawLine = file.readLine();
            if (rawLine.isEmpty()) {
                error = QStringLiteral("PLY vertex data ended after %1 of %2 vertices")
                            .arg(index)
                            .arg(count);
                return false;
            }
            const QStringList tokens = splitFields(QString::fromLatin1(rawLine));
            if (tokens.size() < expected) {
                error = QStringLiteral("PLY vertex %1 has %2 fields, expected %3")
                            .arg(index)
                            .arg(tokens.size())
                            .arg(expected);
                return false;
            }
            double values[3] = {0.0, 0.0, 0.0};
            const int coordinates[3] = {xIndex, yIndex, zIndex};
            for (int axis = 0; axis < 3; ++axis) {
                bool ok = false;
                values[axis] = tokens.at(coordinates[axis]).toDouble(&ok);
                if (!ok) {
                    error = QStringLiteral("PLY vertex %1 has a non-numeric coordinate").arg(index);
                    return false;
                }
            }
            cloud.positions.push_back({static_cast<float>(values[0]),
                                       static_cast<float>(values[1]),
                                       static_cast<float>(values[2])});

            if (hasColor) {
                double channels[3] = {0.0, 0.0, 0.0};
                const int components[3] = {redIndex, greenIndex, blueIndex};
                for (int channel = 0; channel < 3; ++channel) {
                    bool ok = false;
                    channels[channel] = tokens.at(components[channel]).toDouble(&ok);
                    if (!ok) {
                        error = QStringLiteral("PLY vertex %1 has a non-numeric color").arg(index);
                        return false;
                    }
                }
                cloud.colors.push_back({toColorByte(channels[0]),
                                        toColorByte(channels[1]),
                                        toColorByte(channels[2])});
            }
        }
        return true;
    }

    const bool littleEndian = format == Format::BinaryLittleEndian;
    const quint64 blockSize = count * static_cast<quint64>(stride);
    const qint64 remaining = file.size() - file.pos();
    if (remaining < 0 || static_cast<quint64>(remaining) < blockSize) {
        error = QStringLiteral("PLY vertex data is truncated: %1 bytes available, %2 required")
                    .arg(remaining < 0 ? 0 : remaining)
                    .arg(blockSize);
        return false;
    }

    QByteArray block;
    if (!readFully(file, blockSize, block)) {
        error = QStringLiteral("cannot read %1 bytes of PLY vertex data").arg(blockSize);
        return false;
    }

    const char* const base = block.constData();
    const auto& properties = element.properties;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
    for (quint64 index = 0; index < count; ++index) {
        const char* const row = base + index * static_cast<quint64>(stride);
        if (!decodeBinaryScalar(row + offsets.at(static_cast<std::size_t>(xIndex)), properties.at(static_cast<std::size_t>(xIndex)).scalar, littleEndian, x)
            || !decodeBinaryScalar(row + offsets.at(static_cast<std::size_t>(yIndex)), properties.at(static_cast<std::size_t>(yIndex)).scalar, littleEndian, y)
            || !decodeBinaryScalar(row + offsets.at(static_cast<std::size_t>(zIndex)), properties.at(static_cast<std::size_t>(zIndex)).scalar, littleEndian, z)) {
            error = QStringLiteral("PLY vertex %1 has an unsupported coordinate type").arg(index);
            return false;
        }
        cloud.positions.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});

        if (hasColor) {
            if (!decodeBinaryScalar(row + offsets.at(static_cast<std::size_t>(redIndex)), properties.at(static_cast<std::size_t>(redIndex)).scalar, littleEndian, red)
                || !decodeBinaryScalar(row + offsets.at(static_cast<std::size_t>(greenIndex)), properties.at(static_cast<std::size_t>(greenIndex)).scalar, littleEndian, green)
                || !decodeBinaryScalar(row + offsets.at(static_cast<std::size_t>(blueIndex)), properties.at(static_cast<std::size_t>(blueIndex)).scalar, littleEndian, blue)) {
                error = QStringLiteral("PLY vertex %1 has an unsupported color type").arg(index);
                return false;
            }
            cloud.colors.push_back({toColorByte(red), toColorByte(green), toColorByte(blue)});
        }
    }
    return true;
}

bool skipElement(QFile& file, const Element& element, Format format, QString& error)
{
    if (element.count == 0) {
        return true;
    }

    if (format == Format::Ascii) {
        for (quint64 index = 0; index < element.count; ++index) {
            if (file.readLine().isEmpty() && file.atEnd()) {
                error = QStringLiteral("PLY element %1 ended after %2 of %3 rows")
                            .arg(element.name)
                            .arg(index)
                            .arg(element.count);
                return false;
            }
        }
        return true;
    }

    bool fixedSize = true;
    int stride = 0;
    for (const Property& property : element.properties) {
        if (property.isList) {
            fixedSize = false;
            break;
        }
        stride += property.scalar.size;
    }

    if (fixedSize) {
        if (stride <= 0) {
            return true;
        }
        if (!advance(file, element.count * static_cast<quint64>(stride))) {
            error = QStringLiteral("PLY element %1 is truncated").arg(element.name);
            return false;
        }
        return true;
    }

    // Rows containing list properties have a per-row size, so they are walked.
    const bool littleEndian = format == Format::BinaryLittleEndian;
    for (quint64 index = 0; index < element.count; ++index) {
        for (const Property& property : element.properties) {
            if (!property.isList) {
                if (!advance(file, static_cast<quint64>(property.scalar.size))) {
                    error = QStringLiteral("PLY element %1 is truncated").arg(element.name);
                    return false;
                }
                continue;
            }
            QByteArray rawCount;
            if (!readFully(file, static_cast<quint64>(property.countType.size), rawCount)) {
                error = QStringLiteral("PLY element %1 is truncated").arg(element.name);
                return false;
            }
            double countValue = 0.0;
            if (!decodeBinaryScalar(rawCount.constData(), property.countType, littleEndian, countValue) || countValue < 0.0) {
                error = QStringLiteral("PLY element %1 has an invalid list count").arg(element.name);
                return false;
            }
            const quint64 items = static_cast<quint64>(countValue);
            if (!advance(file, items * static_cast<quint64>(property.itemType.size))) {
                error = QStringLiteral("PLY element %1 is truncated").arg(element.name);
                return false;
            }
        }
    }
    return true;
}

} // namespace

bool PlyReader::read(const QString& path, PlyCloud& cloud, QString& error)
{
    cloud.positions.clear();
    cloud.colors.clear();
    error.clear();

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        error = QStringLiteral("cannot open %1: %2").arg(path, file.errorString());
        return false;
    }

    Format format = Format::Ascii;
    std::vector<Element> elements;
    if (!parseHeader(file, format, elements, error)) {
        return false;
    }

    const Element* vertexElement = nullptr;
    for (const Element& element : elements) {
        if (element.name == QLatin1String("vertex")) {
            vertexElement = &element;
            break;
        }
    }
    if (vertexElement == nullptr) {
        error = QStringLiteral("PLY file has no vertex element: %1").arg(path);
        return false;
    }
    if (vertexElement->count == 0) {
        error = QStringLiteral("PLY file contains no vertices: %1").arg(path);
        return false;
    }

    for (const Element& element : elements) {
        if (&element == vertexElement) {
            if (!readVertices(file, element, format, cloud, error)) {
                cloud.positions.clear();
                cloud.colors.clear();
                return false;
            }
            continue;
        }
        if (!skipElement(file, element, format, error)) {
            cloud.positions.clear();
            cloud.colors.clear();
            return false;
        }
    }

    if (cloud.positions.empty()) {
        error = QStringLiteral("PLY file contains no vertices: %1").arg(path);
        return false;
    }
    return true;
}