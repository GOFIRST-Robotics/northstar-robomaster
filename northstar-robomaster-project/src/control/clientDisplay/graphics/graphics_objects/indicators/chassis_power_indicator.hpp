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
    }

    void update()
    {
        powerDraw.integer = static_cast<int32_t>(chassis->getChassisPowerDraw());
        powerDraw.calculateNumbers();
        powerDraw.x = X_POSITION - powerDraw.width / 2;

        if (chassis->getChassisPowerDraw() > 100)
        {
            powerDraw.color = UISubsystem::Color::RED_AND_BLUE;
            powerDraw.show();
        }
        else
        {
            powerDraw.hide();
        }
    }

private:
    tap::Drivers* drivers;

    src::chassis::ChassisSubsystem* chassis;

    static constexpr uint16_t X_POSITION =
        100;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 300;   // pixels, all numbers at the same y level on
                                                  // screen
    static constexpr uint16_t LINE_HEIGHT = 100;  // pixels, this is a large number

    IntegerGraphic powerDraw{};
};

}  // namespace src::control::client_display::graphics