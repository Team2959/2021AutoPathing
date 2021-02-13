#pragma once
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>

// Constants
const double kCloseToSameValue = 0.0000001;
const double kBasicNEOMaxVelocity = 5676.0;

// Drivetrain SparkMax CAN IDs =================================
// Left
const int kDrivetrainLeftPrimary = 1;
const int kDrivetrainLeftFollower1 = 2;
const int kDrivetrainLeftFollower2 = 3;
// Right
const int kDrivetrainRightPrimary = 4;
const int kDrivetrainRightFollower1 = 5;
const int kDrivetrainRightFollower2 = 6;
// Shooter SparkMax CAN IDs ===================================
const int kShooterPrimary = 7;
const int kShooterFollower = 8;
// Victor SPX CAN IDs
const int kIntakeVictorSpxCanId = 10;
const int kKickerVictorSpxCanId = 11;
const int kColorWheelVictorSpxCanId = 12;
const int kConveyorVictorSpxCanId = 13;
// Talon SRX CAN IDs
const int kClimbLeftTalonSrxCanId = 14;
const int kClimbRightTalonSrxCanId = 15;

// Pnuematics Control Module
const int kShooterAngleAdjusterPcmId = 0;
const int kColorWheelEngagePcmId = 1;

// Intake Sensor Digital IO ports
const int kNewPowercellSensor = 0;
const int kSecuredPowercellSensor = 1;
const int kKickerSensor = 2;

// Joystick Buttons
// driver
const int kQuickTurn = 2;
// co-pilot
const int kFire = 1;
const int kIntakeToggle = 2;
const int kSetAngle = 3;
const int kEngageColorWheel = 12;
const int kSpinColorWheel = 9;
const int kGoToColor = 8;
const int kClimbExtend = 11;
const int kClimbRetract = 5;
const int kReverseIntake = 10;
const int kReverseConveyor = 7;
const int kReverseKicker = 6;
const int kTurnToTarget = 4;

const double kGearboxRatio = 1.0 / 6.1; // One turn of the wheel is 6.1 turns of the motor
const double kConversionFactor = kGearboxRatio * 6 * M_PI * 0.0254; // pi * wheel diameter * inches to meters

namespace Drive 
{
    const double kRamseteB = 2;
    const double kRamseteZeta = 0.7;
    const auto ks = 0.157_V;
    const auto kv = 1.54 * 1_V * 1_s / 1_m;
    const auto ka = 0.273 * 1_V * 1_s * 1_s / 1_m;
    const double kPDriveVel = 8.5;
}