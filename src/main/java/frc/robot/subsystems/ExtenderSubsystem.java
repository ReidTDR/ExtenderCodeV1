// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private final TalonFX m_extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_ID);
  private final VoltageOut m_voltageOut = new VoltageOut(0);
  private final double initialPosRotations;
  private final double GearRatio = 20;
  private final double LowerBoundRadians = -1;
  private final double UpperBoundRadians = 1;
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
    initialPosRotations = m_extenderMotor.getPosition().getValueAsDouble();
  }

  public double getPosRadians(){
    return (m_extenderMotor.getPosition().getValueAsDouble() - initialPosRotations) / (GearRatio) * 2 * Math.PI;
  }
  
  public void motorStop(){
    m_extenderMotor.setControl(m_voltageOut.withOutput(0));
  }

  public void motorSetVoltage(double speed){
    speed = Math.max(-1, Math.min(1,speed));

    if (getPosRadians() <= LowerBoundRadians && speed < 0 || getPosRadians() >= UpperBoundRadians && speed > 0){
      motorStop();
      speed = 0;
    }
    m_extenderMotor.setControl(m_voltageOut.withOutput(12 * speed * Math.cos(getPosRadians())));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Position", getPosRadians());
    SmartDashboard.putNumber("Initial Position", initialPosRotations);
    SmartDashboard.putNumber("Voltage", m_extenderMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Current", m_extenderMotor.getStatorCurrent().getValueAsDouble());

    }
    public Command setVoltageCommand(DoubleSupplier speed){
      return runEnd(() -> motorSetVoltage(speed.getAsDouble()), this::motorStop);
    }
}
