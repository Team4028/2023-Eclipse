// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carriage extends SubsystemBase {
    private WPI_TalonSRX m_leftCarriage;
    private WPI_TalonSRX m_rightCarriage;
    private static Carriage m_instance;

    /** Creates a new Carriage. */
    public Carriage() {
        m_leftCarriage = new WPI_TalonSRX(8);
        m_rightCarriage = new WPI_TalonSRX(9);

        m_leftCarriage.setInverted(false);
        m_rightCarriage.setInverted(true);
    }

    public void runCarriageIn() {
        m_leftCarriage.set(0.5);
        m_rightCarriage.set(0.5);
    }

    public void runCarriageOut() {
        m_leftCarriage.set(-0.5);
        m_rightCarriage.set(-0.5);
    }

    public void stop() {
        m_leftCarriage.stopMotor();
        m_rightCarriage.stopMotor();
    }

    public static Carriage getInstance() {
        if (m_instance == null) {
            m_instance = new Carriage();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
