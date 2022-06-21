// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
  WPI_TalonSRX m_leftInfeed;
  WPI_TalonSRX m_rightInfeed;
  WPI_TalonSRX m_leftSwitchBlade;
  WPI_TalonSRX m_rightSwitchBlade;

  private static Infeed m_instance;
  /** Creates a new Infeed. */
  public Infeed() {
    m_leftInfeed = new WPI_TalonSRX(11);
    m_rightInfeed = new WPI_TalonSRX(10);

    m_leftInfeed.setInverted(true);
    m_rightInfeed.setInverted(false);

    m_leftSwitchBlade = new WPI_TalonSRX(5);
    m_rightSwitchBlade = new WPI_TalonSRX(6);

    m_leftSwitchBlade.setInverted(false);
    m_rightSwitchBlade.setInverted(true);

    m_leftSwitchBlade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    m_rightSwitchBlade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
  }

  public void runInfeed() {
    m_leftInfeed.set(0.5);
    m_rightInfeed.set(0.5);
  }

  public void runOutfeed() {
    m_leftInfeed.set(-0.5);
    m_rightInfeed.set(-0.5);
  }

  public void runSwitchBladeForward() {
    m_leftSwitchBlade.set(0.2);
    m_rightSwitchBlade.set(0.2);
  }

  public void runSwitchBladeBackward() {
    m_leftSwitchBlade.set(-0.2);
    m_rightSwitchBlade.set(-0.2);
  }

  public void stop() {
    m_leftInfeed.set(0);
    m_rightInfeed.set(0);
    m_leftSwitchBlade.set(0);
    m_rightSwitchBlade.set(0);

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
  }
}
