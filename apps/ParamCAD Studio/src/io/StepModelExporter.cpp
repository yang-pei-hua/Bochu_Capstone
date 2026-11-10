#include "io/StepModelExporter.h"
#include <QCoreApplication>
#include <modeling/StepExporter.h>

#include <QSaveFile>
#include <exception>
#include <ostream>
#include <streambuf>

namespace {
// Stream directly into the transactional file, keeping large STEP exports
// bounded in memory and leaving path encoding to Qt rather than OCCT fopen.
class SaveFileBuffer final : public std::streambuf
{
public:
    explicit SaveFileBuffer(QSaveFile& file) : m_file(file) {}
protected:
    std::streamsize xsputn(const char* data, std::streamsize count) override
    {
        const qint64 written = m_file.write(data, count);
        return written < 0 ? 0 : written;
    }
    int_type overflow(int_type value) override
    {
        if (traits_type::eq_int_type(value, traits_type::eof())) return traits_type::not_eof(value);
        const char byte = traits_type::to_char_type(value);
        return m_file.write(&byte, 1) == 1 ? value : traits_type::eof();
    }
    int sync() override { return m_file.flush() ? 0 : -1; }
private:
    QSaveFile& m_file;
};
}

bool StepModelExporter::write(const TopoDS_Shape& shape, const QString& path, QString& error)
{
    error.clear();
    if (shape.IsNull()) {
        error = QCoreApplication::translate("StepModelExporter", "没有可导出为 STEP 的 CAD 几何。");
        return false;
    }
    if (path.isEmpty()) {
        error = QCoreApplication::translate("StepModelExporter", "请选择 STEP 输出文件。");
        return false;
    }
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        error = file.errorString();
        return false;
    }
    try {
        SaveFileBuffer buffer(file);
        std::ostream output(&buffer);
        std::string failure;
        const bool written = modeling::exportStep(shape, output, &failure);
        output.flush();
        if (!written || !output.good()) {
            error = file.error() == QFileDevice::NoError
                ? QString::fromStdString(failure) : file.errorString();
            if (error.isEmpty()) error = QCoreApplication::translate("StepModelExporter", "无法写入 STEP 输出。");
            file.cancelWriting();
            return false;
        }
        if (!file.commit()) {
            error = file.errorString();
            return false;
        }
        return true;
    } catch (const std::exception& failure) {
        file.cancelWriting();
        error = QString::fromUtf8(failure.what());
        return false;
    }
}
