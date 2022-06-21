// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Infeed;

public class ZeroSwitchblade extends CommandBase {
    private Infeed m_infeed;
    /** Creates a new ZeroSwitchblade. */
    public ZeroSwitchblade(Infeed infeed) {
        m_infeed = infeed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(infeed);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_infeed.zeroSwitchblades();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_infeed.zeroSwitchblades();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        new WaitCommand(0.2).andThen(
            () -> {
                m_infeed.zeroSwitchbladeEncoders();
                m_infeed.stopSwitchblades();
            }
        ).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_infeed.getLeftLimitSwitch() && m_infeed.getRightLimitSwitch();
    }
}
