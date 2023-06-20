// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.beaklib.motor.BeakTalonSRX;
import frc.lib.beaklib.pid.BeakPIDConstants;

public class Infeed extends SubsystemBase {
    private static final int RIGHT_INFEED = 10;
    private static final int LEFT_INFEED = 11;

    private static final int RIGHT_SWITCHBLADE = 6;
    private static final int LEFT_SWITCHBLADE = 5;

    private BeakTalonSRX m_rightInfeed, m_leftInfeed;
    private BeakTalonSRX m_rightSwitchblade, m_leftSwitchblade;

    private boolean m_homing = false;

    private static Infeed m_instance;

    public enum InfeedTargetState {
        NONE(0),
        WIDE(140),
        INFEED(190),
        STORE(30);

        int value;

        private InfeedTargetState(int value) {
            this.value = value;
        }
    }

    InfeedTargetState m_infeedTargetState = InfeedTargetState.NONE;

    /** Creates a new Infeed. */
    public Infeed() {
        m_rightInfeed = new BeakTalonSRX(RIGHT_INFEED);
        m_leftInfeed = new BeakTalonSRX(LEFT_INFEED);

        m_rightSwitchblade = new BeakTalonSRX(RIGHT_SWITCHBLADE);
        m_leftSwitchblade = new BeakTalonSRX(LEFT_SWITCHBLADE);

        configInfeedMotors();
        configSwitchbladeMotors();
    }

    private void configInfeedMotors() {
        m_rightInfeed.restoreFactoryDefault();
        m_leftInfeed.restoreFactoryDefault();

        m_rightInfeed.setInverted(false);
        m_leftInfeed.setInverted(true);
    }

    private void configSwitchbladeMotors() {
        m_rightSwitchblade.restoreFactoryDefault();
        m_leftSwitchblade.restoreFactoryDefault();

        m_rightSwitchblade.setReverseLimitSwitchNormallyClosed(true);
        m_leftSwitchblade.setReverseLimitSwitchNormallyClosed(true);

        m_rightSwitchblade.setBrake(true);
        m_leftSwitchblade.setBrake(true);

        m_rightSwitchblade.setInverted(true);
        m_leftSwitchblade.setInverted(false);

        m_rightSwitchblade.setPID(new BeakPIDConstants(0.5, 0., 5.0), 0);
        m_leftSwitchblade.setPID(new BeakPIDConstants(0.5, 0., 5.), 0);

        m_rightSwitchblade.setAllowedClosedLoopError(10., 0);
        m_leftSwitchblade.setAllowedClosedLoopError(10., 0);
    }

    public void runToPosition() {
        switch (m_infeedTargetState) {
            case WIDE:
                m_infeedTargetState = InfeedTargetState.INFEED;
                break;
            case INFEED:
                m_infeedTargetState = InfeedTargetState.STORE;
                break;
            default:
                m_infeedTargetState = InfeedTargetState.WIDE;
                break;
        }
    }

    public void zeroSwitchblades() {
        m_rightSwitchblade.set(-0.2);
        m_leftSwitchblade.set(-0.2);

        m_homing = true;
    }

    public void zeroSwitchbladeEncoders() {
        m_rightSwitchblade.resetEncoder();
        m_leftSwitchblade.resetEncoder();
    }

    public void stopSwitchblades() {
        m_rightSwitchblade.stop();
        m_leftSwitchblade.stop();

        m_homing = false;
    }

    public boolean getLeftLimitSwitch() {
        return !m_leftSwitchblade.getReverseLimitSwitch().Value;
    }

    public boolean getRightLimitSwitch() {
        return !m_rightSwitchblade.getReverseLimitSwitch().Value;
    }

    public void runInfeed() {
        m_leftInfeed.set(0.5);
        m_rightInfeed.set(0.5);
    }

    public void runOutfeed() {
        m_leftInfeed.set(-0.5);
        m_rightInfeed.set(-0.5);
    }

    public void stop() {
        stopSwitchblades();
        m_leftInfeed.stopMotor();
        m_rightInfeed.stopMotor();
    }

    public Command runInfeedCommand() {
        return startEnd(
                () -> runInfeed(),
                () -> stop());
    }

    public Command runOutfeedCommand() {
        return startEnd(
                () -> runOutfeed(),
                () -> stop());
    }

    public Command zeroCommand() {
        return run(() -> zeroSwitchblades())
                .until(() -> getLeftLimitSwitch() && getRightLimitSwitch())
                .andThen(new WaitCommand(0.2))
                .andThen(
                        runOnce(() -> {
                            zeroSwitchbladeEncoders();
                            stopSwitchblades();
                        }));
    }

    public static Infeed getInstance() {
        if (m_instance == null) {
            m_instance = new Infeed();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_infeedTargetState != InfeedTargetState.NONE && !m_homing) {
            m_rightSwitchblade.setPositionMotorRotations(m_infeedTargetState.value / 360.);
            m_leftSwitchblade.setPositionMotorRotations(m_infeedTargetState.value / 360.);
        }
    }
}
