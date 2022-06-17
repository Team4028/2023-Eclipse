// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonSRX m_FL, m_BL, m_FR, m_BR;

    private MotorControllerGroup m_left, m_right;

    private DifferentialDrive m_drive;

    private static Drivetrain m_instance = new Drivetrain();

    /* Config and initialization */
    public Drivetrain() {
        m_FL = new WPI_TalonSRX(1);
        m_BL = new WPI_TalonSRX(2);
        m_FR = new WPI_TalonSRX(3);
        m_BR = new WPI_TalonSRX(4);

        m_BL.follow(m_FL);
        m_BR.follow(m_FR);

        configMotors();

        m_left = new MotorControllerGroup(m_FL, m_BL);
        m_right = new MotorControllerGroup(m_FR, m_BR);

        m_drive = new DifferentialDrive(
            m_left,
            m_right
        );
    }

    public void configMotors() {
        configNeutralMode();
        configInverted();
    }

    public void configNeutralMode() {
        m_FL.setNeutralMode(NeutralMode.Coast);
        m_BL.setNeutralMode(NeutralMode.Coast);
        m_FR.setNeutralMode(NeutralMode.Coast);
        m_BR.setNeutralMode(NeutralMode.Coast);
    }

    public void configInverted() {
        m_FL.setInverted(true);
        m_BL.setInverted(true);
        m_FR.setInverted(false);
        m_BR.setInverted(false);
    }

    public void drive(double x, double rot) {
        System.out.println(x);
        m_drive.arcadeDrive(x, rot);
        // m_FL.set(0.7 * x - 0.3 * rot);
        // m_FR.set(0.7 * x + 0.3 * rot);
    }

    public static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
