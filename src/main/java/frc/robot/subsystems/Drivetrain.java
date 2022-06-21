// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PIDConstants;
import frc.robot.utilities.drive.BeakDifferentialDrivetrain;
import frc.robot.utilities.drive.RobotPhysics;
import frc.robot.utilities.motor.BeakTalonSRX;

/** Add your docs here. */
public class Drivetrain extends BeakDifferentialDrivetrain {
    private Field2d field = new Field2d();

    private BeakTalonSRX m_FL, m_BL, m_FR, m_BR;

    private static final double kP = 0.05;
    private static final double kD = 0.0;

    private static final int FL_ID = 1;
    private static final int BL_ID = 2;
    private static final int FR_ID = 3;
    private static final int BR_ID = 4;

    private static final double MAX_VELOCITY = Units.feetToMeters(15.0);

    // distance from the right to left wheels on the robot
    private static final double TRACK_WIDTH = 24;
    // distance from the front to back wheels on the robot
    private static final double WHEEL_BASE = 25;

    private static final double WHEEL_DIAMETER = 6.258;
    private static final double GEAR_RATIO = 7.5;

    private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
            1.1161,
            .17294,
            0.087223);
    
    private static final RobotPhysics PHYSICS = new RobotPhysics(
            MAX_VELOCITY,
            0,
            TRACK_WIDTH,
            WHEEL_BASE,
            WHEEL_DIAMETER,
            GEAR_RATIO,
            FEED_FORWARD);

    private static Drivetrain m_instance;

    public Drivetrain() {
        super(
            PHYSICS,
            PIDConstants.Theta.gains
        );

        m_gyro = new AHRS(SPI.Port.kMXP);

        m_odom = new DifferentialDriveOdometry(getGyroRotation2d());

        m_FL = new BeakTalonSRX(FL_ID);
        m_BL = new BeakTalonSRX(BL_ID);
        m_FR = new BeakTalonSRX(FR_ID);
        m_BR = new BeakTalonSRX(BR_ID);

        m_FL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_FR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        //m_BL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        //m_BR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        m_FL.setDistancePerPulse(m_wheelDiameter, 1);
        m_FR.setDistancePerPulse(m_wheelDiameter, 1);

        configMotors();
    }

    public void configMotors() {
        configPID();
        configNeutralMode();
        configInverted();
    }

    public void configPID() {
        m_FL.setPIDF(kP, 0., kD, /*0.044*/0., 0);
        m_BL.setPIDF(kP, 0., kD, /*0.044*/0., 0);
        m_FR.setPIDF(kP, 0., kD, /*0.044*/0., 0);
        m_BR.setPIDF(kP, 0., kD, /*0.044*/0., 0);
    }

    public void configNeutralMode() {
        m_FL.setBrake(false);
        m_BL.setBrake(false);
        m_FR.setBrake(false);
        m_BR.setBrake(false);
    }

    public void configInverted() {
        m_FL.setInverted(true);
        m_BL.setInverted(true);
        m_FR.setInverted(false);
        m_BR.setInverted(false);
    }

    
    public void drive(double x, double y, double rot) {
        DifferentialDriveWheelSpeeds speeds = calcWheelSpeeds(x, rot);

        // double[] velocities = calcDesiredMotorVelocities(m_FL, x, rot);

        // m_FL.setVelocityNU(velocities[0]);
        // System.out.println(FEED_FORWARD.calculate(speeds.leftMetersPerSecond));
        // m_FL.setVelocityNU(velocities[0], FEED_FORWARD.calculate(speeds.leftMetersPerSecond) / 12.0, 0);
        // m_FR.setVelocityNU(velocities[1], FEED_FORWARD.calculate(speeds.rightMetersPerSecond) / 12.0, 0);
        m_FL.setRate(speeds.leftMetersPerSecond, FEED_FORWARD.calculate(speeds.leftMetersPerSecond) / 12.0);
        m_FR.setRate(speeds.rightMetersPerSecond, FEED_FORWARD.calculate(speeds.rightMetersPerSecond) / 12.0);

        // System.out.println(m_FL.getMotorOutputVoltage());
        // m_FL.set(x);
        // m_FR.set(rot);
        // m_FL.set(0.7 * x - 0.3 * rot);
        // m_FR.set(0.7 * x + 0.3 * rot);
    }

    public void driveVolts(double left, double right) {
        m_FL.setVoltage(left);
        m_FR.setVoltage(right);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return super.getWheelSpeeds(m_FL, m_FR);
    }

    public void resetOdometry(Pose2d pose) {
        super.resetOdometry(pose);
        
        m_FL.resetEncoder();
        m_BL.resetEncoder();
        m_FR.resetEncoder();
        m_BR.resetEncoder();
    }

    public static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        updateOdometry(m_FL, m_FR);

        field.setRobotPose(m_pose);
        SmartDashboard.putData(field);
    }
}
