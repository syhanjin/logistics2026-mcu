/**
 * @file    PCCommandHandler.hpp
 * @brief   Pure chassis command processor.
 */
#pragma once

#include "PCProtocol.hpp"

namespace Protocol::CommandHandler
{

bool enqueueLidarPostureFrame(const LidarPostureFrame& frame);
bool enqueueUpperHostControlFrame(const UpperHostControlFrame& frame);
void startTask();

} // namespace Protocol::CommandHandler
