#pragma once

#include "tap/control/command.hpp"

#include "control/clientDisplay/graphics/graphics_objects/indicators/all_robot_health_numbers.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/chassis_orientation_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/countdown.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/hit_ring.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/hopper_lid_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/imu_recalibration_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/lane_assist_lines.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/linear_velocity_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/peeking_lines.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/predicted_remaining_shots_indicator.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/reticle.hpp"
#include "control/clientDisplay/graphics/graphics_objects/indicators/supercap_charge_indicator.hpp"
// #include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
// #include "subsystems/flywheel/FlywheelSubsystem.hpp"
// #include "subsystems/gimbal/GimbalSubsystem.hpp"
// #include "subsystems/indexer/HeroIndexerSubsystem.hpp"
#include "control/clientDisplay/graphics/core/ui_subsystem.hpp"
#include "control/clientDisplay/graphics/graphics_objects/graphics_container.hpp"

#include "drivers.hpp"

namespace control::clientDisplay::graphics
{
class InfantryDrawCommand : public tap::control::Command, GraphicsContainer
{
public:
    InfantryDrawCommand(
        src::Drivers* drivers,
        UISubsystem* ui,
        GimbalSubsystem* gimbal,
        FlywheelSubsystem* flywheel,
        IndexerSubsystem* indexer,
        DrivetrainSubsystem* drivetrain,
        ServoSubsystem* servo)
        : drivers(drivers),
          ui(ui),
          gimbal(gimbal),
          flywheel(flywheel),
          indexer(indexer),
          drivetrain(drivetrain),
          servo(servo)
    {
        addSubsystemRequirement(ui);

        addGraphicsObject(&lane);
        addGraphicsObject(&supercap);
        addGraphicsObject(&orient);
        addGraphicsObject(&peek);
        addGraphicsObject(&lid);
        addGraphicsObject(&reticle);
        addGraphicsObject(&ring);
        addGraphicsObject(&remain);
        addGraphicsObject(&numbers);
        addGraphicsObject(&countdown);
        addGraphicsObject(&velo);
        addGraphicsObject(&recal);
    };

    void initialize() override { ui->setTopLevelContainer(this); };

    void execute() override
    {
        lane.update();
        supercap.update();
        orient.update();
        peek.update();
        lid.update();
        reticle.update();
        ring.update();
        remain.update();
        numbers.update();
        countdown.update();
        velo.update();
        recal.update();
        // logo doesn't need updating
    };

    // ui subsystem won't do anything until its top level container is set, so we are ok to add
    // objects to the command in the constructor
    void end(bool) override{/*ui->setTopLevelContainer(nullptr);*/};

    bool isFinished() const override { return false; };  // never done drawing ui

    const char* getName() const override { return "infantry ui draw command"; }

private:
    src::Drivers* drivers;
    UISubsystem* ui;
    GimbalSubsystem* gimbal;
    FlywheelSubsystem* flywheel;
    IndexerSubsystem* indexer;
    DrivetrainSubsystem* drivetrain;
    ServoSubsystem* servo;

    // add top level graphics objects here and in the constructor
    LaneAssistLines lane{gimbal};
    SupercapChargeIndicator supercap{drivetrain};
    ChassisOrientationIndicator orient{true, drivers, gimbal, drivetrain};
    PeekingLines peek{drivetrain, gimbal};
    HopperLidIndicator lid{servo};
    Reticle reticle{drivers, gimbal, indexer};
    HitRing ring{drivers, gimbal};
    PredictedRemainingShotsIndicator remain{drivers, indexer};
    AllRobotHealthNumbers numbers{drivers};
    Countdown countdown{drivers};
    LinearVelocityIndicator velo{drivetrain};
    ImuRecalibrationIndicator recal{drivers};
};
}  // namespace control::clientDisplay::graphics