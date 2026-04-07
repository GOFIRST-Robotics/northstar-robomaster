#pragma once

#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

namespace control::clientDisplay::graphics
{
// when trying to buy projectiles as soon as the match starts, you can't see the original countdown
// this is drawn to the side so you can still know the countdown
class Countdown : public GraphicsContainer
{
public:
    Countdown(src::Drivers* drivers) : drivers(drivers)
    {
        addGraphicsObject(&number);
        number.x = X_POSITION;
        number.y = Y_POSITION;
        number.height = LINE_HEIGHT;
    }

    void update()
    {
        // if(drivers->remote.keyPressed(Remote::Key::R))
        //     drivers->recal.requestRecalibration();

        if (drivers->refSerial.getRefSerialReceivingData())
        {
            RefSerialData::Rx::GameData gameData = drivers->refSerial.getGameData();
            number.integer = gameData.stageTimeRemaining;
            number.calculateNumbers();
            number.x = X_POSITION - number.width / 2;

            if (gameData.gameStage == RefSerialData::Rx::GameStage::INITIALIZATION)
            {
                number.color = UISubsystem::Color::ORANGE;
                number.show();
            }
            else if (gameData.gameStage == RefSerialData::Rx::GameStage::COUNTDOWN)
            {
                number.color = UISubsystem::Color::CYAN;
                number.show();
            }
            else
            {
                number.hide();
            }
        }
    }

private:
    src::Drivers* drivers;

    static constexpr uint16_t X_POSITION =
        1680;  // pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 610;   // pixels, all numbers at the same y level on
                                                  // screen
    static constexpr uint16_t LINE_HEIGHT = 200;  // pixels, this is a large number

    IntegerGraphic number{};
};

}  // namespace control::clientDisplay::graphics