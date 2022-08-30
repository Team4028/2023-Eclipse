// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.motor.BeakTalonSRX;

public class Elevator extends SubsystemBase {
    private BeakTalonSRX m_elevatorMotor;

    private static final double[] HOLD_GAINS = { .65, 0, 40., 1.0 };
    private static final double[] UP_GAINS = { 1.5, 0., 75, 0.4 };
    private static final double[] DOWN_GAINS = { 0.2, 0., 2., 0.2 };

    private static final int HOLD_SLOT = 0;
    private static final int UP_SLOT = 1;
    private static final int DOWN_SLOT = 2;

    private static final int MM_CRUISE_VELOCITY = 5000;
    private static final int MM_ACCEL = 4000;

    public static final double NU_PER_INCH = (28510 / 78.75);

    private int m_slot = UP_SLOT;

    public enum ElevatorPosition {
        HOME(0),
        SWITCH(30),
        SCALE(60);

        int value;

        private ElevatorPosition(int value) {
            this.value = value;
        }
    }

    private static Elevator m_instance;

    /** Creates a new Elevator. */
    public Elevator() {
        m_elevatorMotor = new BeakTalonSRX(7);

        m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_elevatorMotor.setSensorPhase(true); // inverted

        m_elevatorMotor.setForwardLimitSwitchNormallyClosed(true);
        m_elevatorMotor.setReverseLimitSwitchNormallyClosed(true);

        m_elevatorMotor.setBrake(true);

        m_elevatorMotor.setMotionMagicCruiseVelocity(MM_CRUISE_VELOCITY, 0);
        m_elevatorMotor.setMotionMagicAcceleration(MM_ACCEL, 0);

        m_elevatorMotor.setPIDF(HOLD_GAINS[0], HOLD_GAINS[1], HOLD_GAINS[2], HOLD_GAINS[3], HOLD_SLOT);
        m_elevatorMotor.setPIDF(UP_GAINS[0], UP_GAINS[1], UP_GAINS[2], UP_GAINS[3], UP_SLOT);
        m_elevatorMotor.setPIDF(DOWN_GAINS[0], DOWN_GAINS[1], DOWN_GAINS[2], DOWN_GAINS[3], DOWN_SLOT);
    }

    public void runToPos() {
        m_elevatorMotor.setMotionMagicNU(30 * NU_PER_INCH, UP_SLOT);
    }

    

    public void runElevatorUp() {
        m_elevatorMotor.set(0.4);
    }

    public void stop() {
        m_elevatorMotor.stop();
    }

    public static Elevator getInstance() {
        if (m_instance == null) {
            m_instance = new Elevator();
        }

        return m_instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
