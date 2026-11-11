// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private final TalonFX m_extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_ID);
  private final VoltageOut m_voltageOut = new VoltageOut(0);

  CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(40)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(60)
    .withSupplyCurrentLimitEnable(true);

    VoltageConfigs voltageConfigs = new VoltageConfigs();

  /** Creates a new ExtenderSubsystem. */
  public ExtenderSubsystem() {

    voltageConfigs.PeakForwardVoltage = 12;
    voltageConfigs.PeakReverseVoltage = -12;

    m_extenderMotor.getConfigurator().apply(currentLimitConfigs);
    m_extenderMotor.getConfigurator().apply(voltageConfigs);
  }

  public void motorSetVoltage(double speed){
    speed = Math.max(-1, Math.min(1,speed));
    m_extenderMotor.setControl(m_voltageOut.withOutput(12 * speed));
  }

  public void motorStop(){
    m_extenderMotor.setControl(m_voltageOut.withOutput(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    }
    public Command setVoltageCommand(DoubleSupplier speed){
      return runEnd(() -> motorSetVoltage(speed.getAsDouble()), this::motorStop);
    }
}
