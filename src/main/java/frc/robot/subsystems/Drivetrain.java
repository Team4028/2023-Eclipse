// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.beaklib.drive.BeakDifferentialDrivetrain;
import frc.lib.beaklib.drive.RobotPhysics;
import frc.lib.beaklib.drive.swerve.DrivetrainConfiguration;
import frc.lib.beaklib.gyro.BeakNavX;
import frc.lib.beaklib.motor.BeakMotorControllerGroup;
import frc.lib.beaklib.motor.BeakTalonSRX;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Acceleration;
import frc.lib.beaklib.units.AngularVelocity;
import frc.lib.beaklib.units.Distance;
import frc.lib.beaklib.units.Velocity;

public class Drivetrain extends BeakDifferentialDrivetrain {
    private static Drivetrain m_instance;

    private static final double kP = 0.05;
    private static final double kD = 0.0;

    private static final BeakPIDConstants DRIVE_PID = new BeakPIDConstants(kP, 0., kD);
    private static final BeakPIDConstants THETA_PID = new BeakPIDConstants(4.5, 0., 0.15);
    private static final BeakPIDConstants AUTON_PID = new BeakPIDConstants(5.0);

    private static final int FL_ID = 1;
    private static final int BL_ID = 2;
    private static final int FR_ID = 3;
    private static final int BR_ID = 4;

    private static final Velocity MAX_VELOCITY = Velocity.fromFeetPerSecond(15.0);

    // distance from the right to left wheels on the robot
    private static final Distance TRACK_WIDTH = Distance.fromInches(24);
    // distance from the front to back wheels on the robot
    private static final Distance WHEEL_BASE = Distance.fromInches(25);

    private static final Distance WHEEL_DIAMETER = Distance.fromInches(6.258);
    private static final double GEAR_RATIO = 7.5;

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
        1.1161,
        .17294,
        0.087223);

    private static final RobotPhysics PHYSICS = new RobotPhysics(
        MAX_VELOCITY,
        new AngularVelocity(),
        new Acceleration(MAX_VELOCITY.getAsMetersPerSecond()),
        TRACK_WIDTH,
        WHEEL_BASE,
        WHEEL_DIAMETER,
        GEAR_RATIO,
        FEED_FORWARD);

    private static final double ALLOWED_CLOSED_LOOP_ERROR = 40.0;

    private static final int DRIVE_SUPPLY_LIMIT = 60;

    private static final DrivetrainConfiguration CONFIG = new DrivetrainConfiguration(
        DRIVE_PID,
        null,
        false,
        ALLOWED_CLOSED_LOOP_ERROR,
        0,
        DRIVE_SUPPLY_LIMIT,
        0,
        "",
        FEED_FORWARD,
        PHYSICS);

    private final BeakNavX m_gyro = new BeakNavX(SPI.Port.kMXP);

    private final BeakTalonSRX m_FL, m_BL, m_FR, m_BR;
    private final BeakMotorControllerGroup m_left;
    private final BeakMotorControllerGroup m_right;

    /** Creates a new Drivetrain. */
    public Drivetrain() {
        super(CONFIG,
            THETA_PID,
            AUTON_PID,
            AUTON_PID,
            false);
        
        m_FL = new BeakTalonSRX(FL_ID);
        m_FR = new BeakTalonSRX(FR_ID);
        m_BL = new BeakTalonSRX(BL_ID);
        m_BR = new BeakTalonSRX(BR_ID);

        m_left = new BeakMotorControllerGroup(m_FL, m_BL);
        m_right = new BeakMotorControllerGroup(m_FR, m_BR);

        super.setup(m_right, m_left, m_gyro);
    }

    public static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }

        return m_instance;
    }

    @Override
    public void periodic() {
        super.updateOdometry();
    }
}
