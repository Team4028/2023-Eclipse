// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Infeed;

public class RunOutfeed extends CommandBase {
    Infeed m_infeed;
    /** Creates a new RunOutfeed. */
    public RunOutfeed(Infeed infeed) {
        m_infeed = infeed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_infeed);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_infeed.runOutfeed();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_infeed.runOutfeed();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_infeed.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
