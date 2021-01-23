#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

class FollowPathCommand : public frc2::CommandHelper<frc2::CommandBase, FollowPathCommand>
{
public:
    FollowPathCommand(wpi::SmallString<64> path);
    void Initialize() override;
private:
    wpi::SmallString<64> m_path;
};
