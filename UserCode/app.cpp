/**
 * @file    app.cpp
 * @brief   Pure chassis application entry.
 */
#include "chassis/chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "project_parts.hpp"
#include "protocol.hpp"
#include "system.hpp"
#include "tim.h"
#include "watchdog.hpp"

void TIM_Callback_1kHz_1(TIM_HandleTypeDef* htim)
{
    (void)htim;

    Chassis::update_1kHz();
}

void TIM_Callback_1kHz_2(TIM_HandleTypeDef* htim)
{
    (void)htim;

    Device::update_1kHz();
    service::Watchdog::EatAll();
}

void TIM_Callback_100Hz(TIM_HandleTypeDef* htim)
{
    (void)htim;

    Chassis::update_100Hz();
}

namespace Arena
{
double get_usage_ratio();
}

double usage = 0.0;

extern "C" void Init(void* argument)
{
    (void)argument;

    Device::init();
    Chassis::init();
    Protocol::init();

    if (service::Watchdog::isFull())
        Error_Handler();

    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz_1);
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, TIM_Callback_1kHz_2);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);

    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_100Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    osDelay(2000);

    if (Chassis::motion != nullptr && !Chassis::motion->enable())
        Error_Handler();

    Chassis::initStandaloneLocCtrl();

    while (!System::Init::inited())
        osDelay(1);

    osDelay(50);

    Chassis::enable();

    osDelay(1000);

    usage = Arena::get_usage_ratio();

    osThreadExit();
}
