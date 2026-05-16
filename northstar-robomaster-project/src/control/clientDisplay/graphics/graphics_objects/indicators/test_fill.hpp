#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace src::control::client_display::graphics
{
class TestFill : public GraphicsContainer
{
public:
    TestFill()
    {
        // lets make each circle radius 30 (size 60), so 32 by 18 circles, 576 total

        for (int i = 0; i < UISubsystem::SCREEN_WIDTH; i += 2 * R)
        {
            for (int j = 0; j < UISubsystem::SCREEN_HEIGHT; j += 2 * R)
            {
                // using new is bad, this is just for testing
                addGraphicsObject(
                    new UnfilledCircle(UISubsystem::Color::ORANGE, i + R, j + R, R, 5));
            }
        }
    }

private:
    static constexpr int R = 60;
};

}  // namespace src::control::client_display::graphics