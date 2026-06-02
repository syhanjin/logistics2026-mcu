#pragma once

#include "IChassisDef.hpp"
#include "project_parts.hpp"

namespace System::Init
{

inline bool postureReceived = false;
inline chassis::Posture posture{};

extern void initPostureReceive();

inline bool inited()
{
    if constexpr (ProjectParts::NeedUpperHostInitPosture)
        return postureReceived;

    return true;
}

} // namespace System::Init
