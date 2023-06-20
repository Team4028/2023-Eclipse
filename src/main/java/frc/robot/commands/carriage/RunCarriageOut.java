// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.carriage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;

public class RunCarriageOut extends CommandBase {
    Carriage m_carriage;
    /** Creates a new RunCarriage. */
    public RunCarriageOut(Carriage carriage) {
        m_carriage = carriage;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_carriage);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_carriage.runCarriageOut();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_carriage.runCarriageOut();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_carriage.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
