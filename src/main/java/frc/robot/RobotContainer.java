// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.auton.BeakAutonCommand;
import frc.robot.auton.TestPath;
import frc.robot.subsystems.CIMDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.BeakXBoxController;

/** Add your docs here. */
public class RobotContainer {
    private BeakXBoxController m_driverController = new BeakXBoxController(OIConstants.DRIVER);

    // private Drivetrain m_drive = Drivetrain.getInstance();
    private CIMDrivetrain m_drive = CIMDrivetrain.getInstance();

    private static RobotContainer _instance = new RobotContainer();

    private SendableChooser<BeakAutonCommand> _autonChooser = new SendableChooser<BeakAutonCommand>();

    private SlewRateLimiter driveXLimiter;
    private SlewRateLimiter driveRotLimiter;

    public RobotContainer() {
        driveXLimiter = new SlewRateLimiter(4.0);
        driveRotLimiter = new SlewRateLimiter(4.0);
        System.out.println("bruh");

        configureButtonBindings();
        initAutonChooser();
    }

    public void configureButtonBindings() {
        System.out.println("Bruh3");
        m_drive.setDefaultCommand(
                new RunCommand(() -> m_drive.drive(
                        driveXLimiter.calculate(m_driverController.getLeftYAxis()),
                        0,
                        driveRotLimiter.calculate(-m_driverController.getRightXAxis())),
                        m_drive
                    ));
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

    // public double speedScaledDriverLeftY() {
    //     return Util.speedScale(m_driverController.getLeftYAxis(),
    //             0.25,
    //             m_driverController.getRightTrigger());
    // }

    // public double speedScaledDriverRightX() {
    //     return -Util.speedScale(m_driverController.getRightXAxis(),
    //             0.25,
    //             m_driverController.getRightTrigger());
    // }

    public static RobotContainer getInstance() {
        return _instance;
    }
}
