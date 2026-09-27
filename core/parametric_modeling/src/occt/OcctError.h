#pragma once

#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>

#include <string>

namespace modeling::occt {

inline std::string failureMessage(const Standard_Failure& failure) {
#if OCC_VERSION_HEX >= 0x080000
    return failure.what();
#else
    const char* message = failure.GetMessageString();
    return message == nullptr ? std::string{} : std::string(message);
#endif
}

}  // namespace modeling::occt
