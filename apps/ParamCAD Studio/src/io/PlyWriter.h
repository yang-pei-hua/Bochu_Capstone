#pragma once
#include "io/PlyReader.h"

class PlyWriter
{
  public:
    static bool write(const PlyCloud& cloud, const QString& path, QString& error);
};
