#include "io/PlyWriter.h"
#include <QDataStream>
#include <QSaveFile>

bool PlyWriter::write(const PlyCloud& cloud, const QString& path, QString& error)
{
    const bool normals = cloud.normals.size() == cloud.positions.size() && !cloud.positions.empty();
    const bool colors = cloud.colors.size() == cloud.positions.size() && !cloud.positions.empty();
    QByteArray header = "ply\nformat binary_little_endian 1.0\ncomment ParamCAD edited working "
                        "cloud\nelement vertex " +
                        QByteArray::number(static_cast<qulonglong>(cloud.positions.size())) +
                        "\nproperty float x\nproperty float y\nproperty float z\n";
    if (normals)
        header += "property float nx\nproperty float ny\nproperty float nz\n";
    if (colors)
        header += "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    header += "end_header\n";
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(header) != header.size()) {
        error = file.errorString();
        return false;
    }
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    for (size_t i = 0; i < cloud.positions.size(); ++i) {
        for (float coordinate : cloud.positions[i])
            stream << coordinate;
        if (normals)
            for (float component : cloud.normals[i])
                stream << component;
        if (colors)
            for (unsigned char component : cloud.colors[i])
                stream << static_cast<quint8>(component);
    }
    if (stream.status() != QDataStream::Ok || !file.commit()) {
        error = file.errorString();
        return false;
    }
    return true;
}
