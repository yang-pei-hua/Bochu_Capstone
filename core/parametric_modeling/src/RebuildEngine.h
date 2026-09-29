#pragma once

namespace modeling {

class PartDocument;

class RebuildEngine {
public:
    bool rebuild(PartDocument& document) const;
};

}  // namespace modeling
