/**
 * @file    PCFeedback.hpp
 * @brief   Pure chassis feedback task.
 */
#pragma once

#include "PCProtocol.hpp"

namespace Protocol::Feedback
{

void registerProtocol(PCProtocol* protocol);
void startTask();

} // namespace Protocol::Feedback
