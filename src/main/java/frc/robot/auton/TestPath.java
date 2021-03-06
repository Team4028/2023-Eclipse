// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import frc.robot.utilities.Util;
import frc.robot.utilities.drive.BeakDifferentialDrivetrain;
import frc.robot.utilities.drive.Trajectories;

public class TestPath extends BeakAutonCommand {
    /** Creates a new TestPath. */
    public TestPath(BeakDifferentialDrivetrain drivetrain) {
        super.addCommands(
                Util.getTrajectoryCommand(Trajectories.TestPath1(drivetrain), drivetrain));
                // new RotateDrivetrainToAngle(Rotation2d.fromDegrees(180.), drivetrain, false));
        super.setInitialPose(Trajectories.TestPath1(drivetrain));
    }
}
