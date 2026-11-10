#include "reconstruction/settings/ModuleSettings.h"

#include <cctype>
#include <cstdint>
#include <mutex>
#include <utility>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

namespace reconstruction::settings {
namespace {

std::filesystem::path g_configDirectory;
bool g_hasConfigDirectory = false;
std::vector<std::string> g_overrides;
std::mutex g_overrideMutex;

std::filesystem::path executableDirectory()
{
#if defined(_WIN32)
    std::wstring path(32768, L'\0');
    const auto count = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (count && count < path.size()) {
        path.resize(count);
        return std::filesystem::path(path).parent_path();
    }
#elif defined(__APPLE__)
    std::uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::string path(size, '\0');
    if (_NSGetExecutablePath(path.data(), &size) == 0)
        return std::filesystem::path(path.c_str()).parent_path();
#elif defined(__linux__)
    std::error_code error;
    return std::filesystem::read_symlink("/proc/self/exe", error).parent_path();
#endif
    return {};
}

}  // namespace

void setConfigDirectory(std::filesystem::path directory)
{
    g_configDirectory = std::move(directory);
    g_hasConfigDirectory = true;
}

void clearConfigDirectory()
{
    g_configDirectory.clear();
    g_hasConfigDirectory = false;
}

std::filesystem::path configDirectory()
{
    if (g_hasConfigDirectory) return g_configDirectory;
    return executableDirectory() / "config";
}

std::string environmentName(std::string_view id)
{
    std::string name = "PARAMCAD_";
    for (const char character : id) {
        if (std::isalnum(static_cast<unsigned char>(character)))
            name.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(character))));
        else
            name.push_back('_');
    }
    name += "_CONFIG";
    return name;
}

std::filesystem::path configPath(std::string_view id, std::string_view filename)
{
    const auto variable = environmentName(id);
#if defined(_WIN32)
    const std::wstring wide(variable.begin(), variable.end());
    if (const auto* value = _wgetenv(wide.c_str()); value && *value)
        return std::filesystem::path(value);
#else
    if (const auto* value = std::getenv(variable.c_str()); value && *value)
        return std::filesystem::u8path(value);
#endif
    std::error_code error;
    const auto installed = configDirectory() / std::filesystem::path(filename);
    if (std::filesystem::exists(installed, error)) return installed;
#ifdef PARAMCAD_CONFIG_SOURCE_DIR
    const auto source = std::filesystem::path(PARAMCAD_CONFIG_SOURCE_DIR) / std::filesystem::path(filename);
    if (std::filesystem::exists(source, error)) return source;
#endif
    return installed;
}

void addOverride(std::string_view assignment)
{
    std::lock_guard<std::mutex> lock(g_overrideMutex);
    g_overrides.emplace_back(assignment);
}

void clearOverrides()
{
    std::lock_guard<std::mutex> lock(g_overrideMutex);
    g_overrides.clear();
}

const std::vector<std::string>& overrides()
{
    return g_overrides;
}

}  // namespace reconstruction::settings
