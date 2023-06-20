// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.beaklib.BeakXBoxController;
import frc.robot.commands.auton.BeakAutonCommand;
import frc.robot.commands.carriage.RunCarriageIn;
import frc.robot.commands.carriage.RunCarriageOut;
import frc.robot.commands.infeed.RunInfeed;
import frc.robot.commands.infeed.RunOutfeed;
import frc.robot.commands.infeed.ZeroSwitchblade;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Infeed;

/** Add your docs here. */
public class RobotContainer {
    private BeakXBoxController m_driverController = new BeakXBoxController(0);

    private Drivetrain m_drive = Drivetrain.getInstance();
    private Infeed m_infeed = Infeed.getInstance();
    private Carriage m_carriage = Carriage.getInstance();
    private Elevator m_elevator = Elevator.getInstance();

    private static RobotContainer _instance = new RobotContainer();

    private SendableChooser<BeakAutonCommand> _autonChooser = new SendableChooser<BeakAutonCommand>();

    private SlewRateLimiter driveXLimiter;
    private SlewRateLimiter driveRotLimiter;

    public RobotContainer() {
        driveXLimiter = new SlewRateLimiter(4.0);
        driveRotLimiter = new SlewRateLimiter(4.0);

        configureButtonBindings();
        initAutonChooser();
    }

    public void configureButtonBindings() {
        m_driverController.a.onTrue(new ZeroSwitchblade(m_infeed));
        m_driverController.b.onTrue(() -> m_infeed.runToPosition());

        m_driverController.lt.whileTrue(new RunCarriageIn(m_carriage));
        m_driverController.lt.whileTrue(new RunInfeed(m_infeed));

        m_driverController.lb.toggleOnTrue(new RunCarriageOut(m_carriage));
        m_driverController.lb.toggleOnTrue(new RunOutfeed(m_infeed));

        m_driverController.y.onTrue(new RunElevator(m_elevator));

        m_driverController.dpad.up.onTrue(new BumpElevatorUp(true, m_elevator));
        m_driverController.dpad.down.onTrue(new BumpElevatorDown(true, m_elevator));

        m_drive.setDefaultCommand(
                new RunCommand(() -> m_drive.drive(
                        driveXLimiter.calculate(m_driverController.getLeftYAxis()),
                        0,
                        driveRotLimiter.calculate(-m_driverController.getRightXAxis())),
                        m_drive));
    }

    private void initAutonChooser() {
        _autonChooser.setDefaultOption("Test Path 1", new TestPath(m_drive));

        SmartDashboard.putData("Auton Chooser", _autonChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_drive.resetOdometry(_autonChooser.getSelected().getInitialPose());
        return _autonChooser.getSelected();
    }

    public static RobotContainer getInstance() {
        return _instance;
    }
}
