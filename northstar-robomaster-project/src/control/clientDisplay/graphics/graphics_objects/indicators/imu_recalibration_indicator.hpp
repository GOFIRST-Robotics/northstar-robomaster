// #pragma once

// #include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
// #include "control/clientDisplay/graphics/graphics_objects/atomic_graphics_objects.hpp"
// #include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

// namespace src::control::client_display::graphics
// {
// // press ctrl r to schedule a imu recalibration just before the match starts
// // press r to unschedule it
// class ImuRecalibrationIndicator : public GraphicsContainer
// {
// public:
//     ImuRecalibrationIndicator(src::Drivers* drivers) : drivers(drivers)
//     {
//         addGraphicsObject(&line1);
//         addGraphicsObject(&line2);
//     }

//     void update()
//     {
//         if (!ignoreKeyPresses && drivers->remote.keyPressed(Remote::Key::CTRL) &&
//             drivers->remote.keyPressed(Remote::Key::SHIFT) &&
//             drivers->remote.keyPressed(Remote::Key::R))
//         {
//             drivers->recal.forceCalibration();
//             ignoreKeyPresses = true;
//             line1.setString("Forcing recal");
//             line2.setString("");
//         }

//         if (drivers->recal.getState() ==
//                 src::ImuRecalibration::ImuRecalibrationState::AFTER_FIRST_CALIBRATION &&
//             isInPrematch())
//         {
//             line1.setString("CTRL+R to schedule");
//             line2.setString("imu recalibration");
//             line1.color = colorUnscheduled;
//             line2.color = colorUnscheduled;

//             if (!ignoreKeyPresses && drivers->remote.keyPressed(Remote::Key::CTRL) &&
//                 drivers->remote.keyPressed(Remote::Key::R))
//             {
//                 drivers->recal.requestRecalibration();
//                 ignoreKeyPresses = true;
//                 update();
//             }
//         }
//         else if (
//             drivers->recal.getState() ==
//                 src::ImuRecalibration::ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED &&
//             isInPrematch())
//         {
//             line1.setString("Imu recal scheduled");
//             line2.setString("R to cancel");
//             line1.color = colorScheduled;
//             line2.color = colorScheduled;

//             if (!ignoreKeyPresses && drivers->remote.keyPressed(Remote::Key::R))
//             {
//                 drivers->recal.cancelRequestRecalibration();
//                 ignoreKeyPresses = true;
//                 update();
//             }
//         }
//         else if (
//             drivers->recal.getState() ==
//             src::ImuRecalibration::ImuRecalibrationState::SECOND_CALIBRATION_WAITING_TO_START)
//         {
//             line1.setString("Waiting for robot");
//             line2.setString("to settle");
//             line1.color = colorWaiting;
//             line2.color = colorWaiting;
//         }
//         else if (
//             drivers->recal.getState() ==
//             src::ImuRecalibration::ImuRecalibrationState::SECOND_CALIBRATION_JUST_BEFORE_START)
//         {
//             line1.setString("Recalibrating. Robot");
//             line2.setString("might move on startup!");
//             line1.color = colorRecalibrating;
//             line2.color = colorRecalibrating;
//         }
//         else if (
//             drivers->recal.getState() ==
//                 src::ImuRecalibration::ImuRecalibrationState::AFTER_SECOND_CALIBRATION &&
//             !(drivers->refSerial.getRefSerialReceivingData() &&
//               drivers->refSerial.getGameData().gameStage ==
//                   RefSerialData::Rx::GameStage::COUNTDOWN))
//         {
//             line1.setString("Done Recalibrating");
//             line2.setString("Good Luck");
//             line1.color = colorDoneRecalibrating;
//             line2.color = colorDoneRecalibrating;
//         }
//         else
//         {
//             line1.hide();
//             line2.hide();
//         }

//         // when in a best of 3 (or best of 2) game, we allow third and fourth calibrations
//         if (isInPrematch())
//         {
//             drivers->recal.allowAnotherRecalibration();
//             line1.show();
//             line2.show();
//         }

//         if (!drivers->remote.keyPressed(Remote::Key::R)) ignoreKeyPresses = false;
//     }

// private:
//     src::Drivers* drivers;

//     static constexpr uint16_t X_POSITION = 40;   // pixels
//     static constexpr uint16_t Y_POSITION = 550;  // pixels
//     static constexpr uint16_t LINE_HEIGHT = 20;  // pixels
//     static constexpr uint16_t THICKNESS = 2;     // pixels

//     static constexpr UISubsystem::Color colorUnscheduled = UISubsystem::Color::CYAN;
//     static constexpr UISubsystem::Color colorScheduled = UISubsystem::Color::ORANGE;
//     static constexpr UISubsystem::Color colorRecalibrating = UISubsystem::Color::PURPLISH_RED;
//     static constexpr UISubsystem::Color colorWaiting = UISubsystem::Color::YELLOW;
//     static constexpr UISubsystem::Color colorDoneRecalibrating = UISubsystem::Color::GREEN;

//     StringGraphic line1{
//         UISubsystem::Color::CYAN,
//         "l1",
//         X_POSITION,
//         Y_POSITION + 2 * LINE_HEIGHT + LINE_HEIGHT / 2,
//         LINE_HEIGHT,
//         THICKNESS};
//     StringGraphic
//         line2{UISubsystem::Color::CYAN, "l2", X_POSITION, Y_POSITION, LINE_HEIGHT, THICKNESS};

//     bool ignoreKeyPresses = false;

//     bool isInPrematch()
//     {
//         return drivers->refSerial.getRefSerialReceivingData() &&
//                (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::SETUP
//                ||
//                 drivers->refSerial.getGameData().gameStage ==
//                     RefSerialData::Rx::GameStage::PREMATCH);
//     }
// };

// }  // namespace src::control::client_display::graphics