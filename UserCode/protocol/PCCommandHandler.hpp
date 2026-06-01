/**
 * @file    PCCommandHandler.hpp
 * @brief   Pure chassis command processor.
 */
#pragma once

#include "PCProtocol.hpp"

namespace Protocol::CommandHandler
{

bool enqueueFrame(const Frame& frame);
void startTask();

} // namespace Protocol::CommandHandler
