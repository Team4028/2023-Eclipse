// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.beaklib.motor.BeakTalonSRX;
import frc.lib.beaklib.pid.BeakPIDConstants;
import frc.lib.beaklib.units.Distance;

// 35:1
// bruh: 1.79"
public class Elevator extends SubsystemBase {
    private BeakTalonSRX m_elevatorMotor;

    private static final BeakPIDConstants HOLD_GAINS = new BeakPIDConstants(.65, 0, 40., 1.0);
    private static final BeakPIDConstants UP_GAINS = new BeakPIDConstants(1.5, 0., 75, 0.4);
    private static final BeakPIDConstants DOWN_GAINS = new BeakPIDConstants(0.2, 0., 2., 0.2);

    private static final int HOLD_SLOT = 0;
    private static final int UP_SLOT = 1;
    private static final int DOWN_SLOT = 2;

    private static final int MM_CRUISE_VELOCITY = 5000;
    private static final int MM_ACCEL = 4000;

    private static final double NU_PER_INCH = (28510 / 78.75); // what

    private static final double SMALL_BUMP = 1.;
    private static final double BIG_BUMP = 2.;

    public enum ElevatorPosition {
        HOME(0),
        SWITCH(30),
        SCALE(75);

        int value;

        private ElevatorPosition(int value) {
            this.value = value;
        }
    }

    private ElevatorPosition m_targetPresetPosition = ElevatorPosition.HOME;

    private double m_targetPosition;

    private static Elevator m_instance;

    /** Creates a new Elevator. */
    public Elevator() {
        m_elevatorMotor = new BeakTalonSRX(7);

        m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_elevatorMotor.setSensorPhase(true); // inverted

        m_elevatorMotor.setForwardLimitSwitchNormallyClosed(true);
        m_elevatorMotor.setReverseLimitSwitchNormallyClosed(true);

        m_elevatorMotor.setBrake(true);

        m_elevatorMotor.setMotionMagicCruiseVelocity(MM_CRUISE_VELOCITY);
        m_elevatorMotor.setMotionMagicAcceleration(MM_ACCEL);

        m_elevatorMotor.setSlot(HOLD_SLOT);
        m_elevatorMotor.setPID(HOLD_GAINS);

        m_elevatorMotor.setSlot(DOWN_SLOT);
        m_elevatorMotor.setPID(DOWN_GAINS);

        m_elevatorMotor.setSlot(UP_SLOT);
        m_elevatorMotor.setPID(UP_GAINS);

        // m_elevatorMotor.setEncoderGearRatio(35.);
        m_elevatorMotor.setWheelDiameter(Distance.fromInches(1.79 * 2.)); // 15T #35 sprocket
    }

    public void runToPosition() {
        switch (m_targetPresetPosition) {
            case HOME:
                m_elevatorMotor.setSlot(UP_SLOT);
                m_targetPresetPosition = ElevatorPosition.SWITCH;
                break;
            case SWITCH:
                m_targetPresetPosition = ElevatorPosition.SCALE;
                break;
            default:
                m_elevatorMotor.setSlot(DOWN_SLOT);
                m_targetPresetPosition = ElevatorPosition.HOME;
                break;
        }

        m_targetPosition = m_targetPresetPosition.value;
    }

    public Command runToPositionCommand() {
        return runOnce(() -> runToPosition());
    }

    public void bumpUp(boolean big) {
        m_targetPosition += big ? BIG_BUMP : SMALL_BUMP;
    }

    public Command bumpUpCommand(boolean big) {
        return runOnce(() -> bumpUp(big));
    }

    public void bumpDown(boolean big) {
        m_targetPosition -= big ? BIG_BUMP : SMALL_BUMP;
    }

    public Command bumpDownCommand(boolean big) {
        return runOnce(() -> bumpDown(big));
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
        m_targetPosition = MathUtil.clamp(m_targetPosition, 0, 80); // FIXME: find max height.
        // m_elevatorMotor.setMotionMagicNU(m_targetPosition * NU_PER_INCH);
        m_elevatorMotor.setMotionMagic(Distance.fromInches(m_targetPosition));
        SmartDashboard.putNumber("Target", m_targetPosition);
        SmartDashboard.putNumber("Believed", m_elevatorMotor.getDistance(false).Value.getAsInches());
        SmartDashboard.putNumber("Actual", m_elevatorMotor.getPositionNU(false).Value / NU_PER_INCH);
        // This method will be called once per scheduler run
    }
}
