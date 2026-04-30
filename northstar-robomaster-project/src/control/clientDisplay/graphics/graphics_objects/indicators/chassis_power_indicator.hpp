#pragma once

#include "control/chassis/chassis_subsystem.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original
// countdown this is drawn to the side so you can still know the countdown
class ChassisPowerIndicator : public GraphicsContainer
{
public:
    ChassisPowerIndicator(tap::Drivers* drivers, src::chassis::ChassisSubsystem* chassis)
        : drivers(drivers),
          chassis(chassis)
    {
        addGraphicsObject(&powerDraw);
        powerDraw.x = X_POSITION;
        powerDraw.y = Y_POSITION;
        powerDraw.height = LINE_HEIGHT;
        addGraphicsObject(&chargeBarOutline);
        addGraphicsObject(&chargeBar);
    }

    void update()
    {
        powerDraw.integer = static_cast<int32_t>(chassis->getChassisPowerDraw());
        powerDraw.calculateNumbers();
        powerDraw.x = X_POSITION - powerDraw.width / 2;

        if (chassis->getChassisPowerDraw() > chassis->getChassisPowerLimit())
        {
            powerDraw.color = UISubsystem::Color::RED_AND_BLUE;
            energyInBuffer -= (chassis->getChassisPowerDraw() - chassis->getChassisPowerLimit()) *
                              drivers->DT / 1000.0f;
        }
        else
        {
            powerDraw.color = UISubsystem::Color::WHITE;
            energyInBuffer += RECHARGE_PER_CYCLE;
        }

        if (energyInBuffer < 0) energyInBuffer = 0;
        if (energyInBuffer > 60) energyInBuffer = 60;
        float chargeBarLength = (energyInBuffer / 60.0f) * 200;
        chargeBar.x2 = X_POSITION - 100 + chargeBarLength;
    }

private:
    tap::Drivers* drivers;

    src::chassis::ChassisSubsystem* chassis;

    static constexpr uint16_t X_POSITION =
        600;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 300;   // pixels, all numbers at the same y level on
                                                  // screen
    static constexpr uint16_t LINE_HEIGHT = 100;  // pixels, this is a large number

    float RECHARGE_PER_CYCLE = 15 * drivers->DT / 1000.0f;

    float energyInBuffer = 60.0f;

    IntegerGraphic powerDraw{};
    UnfilledRectangle chargeBarOutline{
        UISubsystem::Color::RED_AND_BLUE,
        X_POSITION - 300,
        Y_POSITION,
        200,
        LINE_HEIGHT,
        2};
    Line chargeBar{
        UISubsystem::Color::RED_AND_BLUE,
        X_POSITION - 300,
        Y_POSITION,
        X_POSITION - 100,
        Y_POSITION,
        16};
};

}  // namespace src::control::client_display::graphics