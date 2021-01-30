#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/RamseteCommand.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include <subsystems/Drivetrain.h>

class FollowPathCommand : public frc2::CommandHelper<frc2::CommandBase, FollowPathCommand>
{
public:
    FollowPathCommand(Drivetrain & drivetrain, wpi::SmallString<64> path);
    void Initialize() override;
private:
    Drivetrain & m_drivetrain;
    wpi::SmallString<64> m_path;
};
