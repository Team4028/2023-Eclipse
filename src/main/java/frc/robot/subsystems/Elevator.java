// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private WPI_TalonSRX m_elevatorMotor;

    private static Elevator m_instance;

    private static final double NU2INCHES = (28510 / 78.75);

    private static final double[] UP_GAINS = { 1.5, 0., 75, 0.4 };
    private static final double[] DOWN_GAINS = { 0.2, 0., 2., 0.2 };

    public enum ElevatorPosition {
        HOME,
        MID,
        HIGH,
        
    }

    private double m_targetPosition = 0;

    /** Creates a new climber. */
    public Elevator() {

        m_elevatorMotor = new WPI_TalonSRX(7);
        m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        m_elevatorMotor.setSelectedSensorPosition(0, 0, 0);
        m_elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 0);
        m_elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 0);
        m_elevatorMotor.setSensorPhase(true);

        m_elevatorMotor.config_kP(0, UP_GAINS[0]);
        m_elevatorMotor.config_kD(0, UP_GAINS[2]);
        m_elevatorMotor.config_kF(0, UP_GAINS[3]);

        m_elevatorMotor.config_kP(1, DOWN_GAINS[0]);
        m_elevatorMotor.config_kD(1, DOWN_GAINS[2]);
        m_elevatorMotor.config_kF(1, DOWN_GAINS[3]);


    }

    public void runClimb() {
        m_elevatorMotor.set(ControlMode.PercentOutput, 0.35);
    }

    public void stopClimb() {
        m_elevatorMotor.set(ControlMode.PercentOutput, 0);
    }

    public void midClimb() {
        m_targetPosition = 20;
    }

    public void highClimb() {
        m_targetPosition = 40;
    }

    public void runToPosition(double inches) {
        m_elevatorMotor.set(ControlMode.MotionMagic, inchesToNativeUnits(inches));
    }

    public static Elevator getInstance() {
        if (m_instance == null) {
            m_instance = new Elevator();
        }
        return m_instance;
    }

    private static double nativeUnitsToInches(double nu) {
        return (nu / NU2INCHES);
    }

    private static int inchesToNativeUnits(double inches) {
        return ((int) (inches * NU2INCHES));
    }

    @Override
    public void periodic() {
        if (m_elevatorMotor.getSensorCollection().isRevLimitSwitchClosed() == false
                && m_elevatorMotor.getSelectedSensorPosition(0) != 0) {
            m_elevatorMotor.setSelectedSensorPosition(0, 0, 0);
        }

        runToPosition(m_targetPosition); 

        System.out.println(
                "Current encoder position: " + nativeUnitsToInches(m_elevatorMotor.getSelectedSensorPosition(0)));
        // This method will be called once per scheduler run
    }
}
