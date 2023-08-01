// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakTalonSRX;

public class Carriage extends SubsystemBase {
    private BeakTalonSRX m_leftCarriage;
    private BeakTalonSRX m_rightCarriage;

    private static Carriage m_instance;

    /** Creates a new Carriage. */
    public Carriage() {
        m_leftCarriage = new BeakTalonSRX(8);
        m_rightCarriage = new BeakTalonSRX(9);

        m_leftCarriage.setInverted(false);
        m_rightCarriage.setInverted(true);
    }

    public void runCarriage(double vbus) {
        m_leftCarriage.set(vbus);
        m_rightCarriage.set(vbus);
    }

    public Command runCarriageCommand(double vbus) {
        return startEnd(
                () -> runCarriage(vbus),
                () -> runCarriage(0.));
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
