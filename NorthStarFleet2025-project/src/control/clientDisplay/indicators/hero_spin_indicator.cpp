/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hero_spin_indicator.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace src::control::client_display
{
HeroSpinIndicator::HeroSpinIndicator(
    RefSerialTransmitter &refSerialTransmitter,
    const RefSerial &refSerial,
    tap::Drivers &drivers,
    chassis::ChassisBeybladeCommand *beyblade,
    chassis::ChassisWiggleCommand *wiggle)
    : HudIndicator(refSerialTransmitter),
      refSerial(refSerial),
      drivers(drivers),
      beyblade(beyblade),
      wiggle(wiggle)

{
}

modm::ResumableResult<void> HeroSpinIndicator::update()
{
    if (drivers.commandScheduler.isCommandScheduled(beyblade) ||
        drivers.commandScheduler.isCommandScheduled(wiggle))
    {
        circleColor = Tx::GraphicColor::GREEN;
    }
    else
    {
        circleColor = Tx::GraphicColor::BLACK;
    }

    RefSerialTransmitter::configGraphicGenerics(
        &circleGraphics.graphicData,
        circleName,
        Tx::GRAPHIC_MODIFY,
        DEFAULT_GRAPHIC_LAYER,
        circleColor);

    RF_BEGIN(1);

    RF_CALL(refSerialTransmitter.sendGraphic(&circleGraphics));

    RF_END();
}

modm::ResumableResult<void> HeroSpinIndicator::sendInitialGraphics()
{
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&textGraphic));

    RF_CALL(refSerialTransmitter.sendGraphic(&circleGraphics));

    RF_END();
}

void HeroSpinIndicator::initialize()
{
    uint8_t graphicName[3];

    getUnusedGraphicName(graphicName);
    RefSerialTransmitter::configGraphicGenerics(
        &textGraphic.graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::ORANGE);

    RefSerialTransmitter::configCharacterMsg(
        SIZE,
        WIDTH,
        TEXT_X,
        TEXT_Y,
        "Spinning ",
        &textGraphic);

    getUnusedGraphicName(circleName);
    RefSerialTransmitter::configGraphicGenerics(
        &circleGraphics.graphicData,
        circleName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        circleColor);
}

}  // namespace src::control::client_display
