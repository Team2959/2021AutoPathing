
#include <commands/FollowPathCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>

FollowPathCommand::FollowPathCommand(Drivetrain & drivetrain, wpi::SmallString<64> path)
    : m_drivetrain(drivetrain), m_path(path)
{
}

void FollowPathCommand::Initialize()
{

}
