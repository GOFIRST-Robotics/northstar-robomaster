#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"
// #include "subsystems/servo/ServoSubsystem.hpp"

using namespace tap::communication::serial;
namespace control::clientDisplay::graphics
{
class HopperLidIndicator : public GraphicsContainer
{
public:
    HopperLidIndicator(ServoSubsystem* servo) : servo(servo)
    {
        addGraphicsObject(&lid);
        addGraphicsObject(&frame);
    }

    void update()
    {
        Vector2d one{0, HEIGHT / 2};
        Vector2d two{WIDTH, HEIGHT / 2};
        if (servo->getTargetIsClosed())
        {
            // leave vectors unrotated
        }
        else
        {
            one = one.rotate(OPEN_ROTATION_AMOUNT);
            two = two.rotate(OPEN_ROTATION_AMOUNT);
        }
        lid.x1 = one.getX() + CENTER_X;
        lid.y1 = one.getY() + CENTER_Y;
        lid.x2 = two.getX() + CENTER_X;
        lid.y2 = two.getY() + CENTER_Y;
    }

private:
    ServoSubsystem* servo;

    Line lid{UISubsystem::Color::GREEN, 0, 0, 0, 0, HEIGHT};
    UnfilledRectangle frame{
        UISubsystem::Color::WHITE,
        CENTER_X - FRAME_THICKNESS / 2,
        CENTER_Y - FRAME_THICKNESS / 2,
        WIDTH + FRAME_THICKNESS,
        HEIGHT + FRAME_THICKNESS,
        FRAME_THICKNESS};

    static constexpr uint16_t CENTER_X = 1250;  // puts it on top of )
    static constexpr uint16_t CENTER_Y = 810;   // puts it on top of )

    static constexpr float OPEN_ROTATION_AMOUNT = 1;

    static constexpr uint16_t HEIGHT = 20;
    static constexpr uint16_t WIDTH = 30;
    static constexpr uint16_t FRAME_THICKNESS = 2;
};

}  // namespace control::clientDisplay::graphics