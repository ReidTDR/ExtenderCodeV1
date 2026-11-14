// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private final TalonFX m_extenderMotor = new TalonFX(Constants.EXTENDER_MOTOR_ID);
  private final VoltageOut m_voltageOut = new VoltageOut(0); //Starting output voltage (Should be 0)
  private final double initialPosRotations;
  private final double GearRatio = 20;
  private final double LowerBoundRadians = Math.toRadians(-81); //-81 Degrees
  private final double UpperBoundRadians = Math.toRadians(81); //81 Degrees
  private final double ExtenderLength = 0.381; //Meters
  private final double TotalExtenderWeight = 9; //Kg
  private final double kG = 2.25; //Volts to hold against gravity when angle is horizontal
  CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(80)
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(100)
    .withSupplyCurrentLimitEnable(true);

    VoltageConfigs voltageConfigs = new VoltageConfigs();


  public final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0); //Creates an instance for motor position
  
  public final Mechanism2d armMechanism2d = new Mechanism2d(4, 4); //Creates a box w/ dimensions 3Meters x 3Meters
  public final MechanismRoot2d armRoot = armMechanism2d.getRoot("Arm Root", 0.75, 1.5); //Sets the "Moutning Point" of the mechanism
  public final MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Extender Arm", 1.5, 0)); //Creates the ligament


  /** Creates a new ExtenderSubsystem. */
  public ExtenderSubsystem() {

    voltageConfigs.PeakForwardVoltage = 12;
    voltageConfigs.PeakReverseVoltage = -12;

    m_extenderMotor.getConfigurator().apply(currentLimitConfigs);
    m_extenderMotor.getConfigurator().apply(voltageConfigs);
    m_extenderMotor.setNeutralMode(NeutralModeValue.Brake);
    initialPosRotations = m_extenderMotor.getPosition().getValueAsDouble();

    Slot0Configs slot0 = new Slot0Configs(); //Sets values used for m_PositionVoltage
      slot0.kP = 1.25;
      slot0.kD = 1;
      m_extenderMotor.getConfigurator().apply(slot0); //Applies Slot0
  }

  public double getPosRadians(){
    return (m_extenderMotor.getPosition().getValueAsDouble() - initialPosRotations) / (GearRatio) * 2 * Math.PI; // Gets current motor position, finds output shaft rotations, and then converts to radians
  }
  
  public void motorStop(){ //Sets volts to 0 and sets speed to 0 to prevent movement after cutting the input
    m_extenderMotor.setControl(m_voltageOut.withOutput(0));
    motorSetVoltage(0);
    m_extenderMotor.stopMotor();
  }

  public void motorSetVoltage(double speed){ 
    speed = Math.max(-1, Math.min(1,speed)); //Speed ranges from -1 to 1
    double volts = speed > 0 ? 
    12 * speed + kG * Math.cos(getPosRadians()) 
    : (speed <= 0 ?
    12 * speed + 5 * kG * Math.cos(getPosRadians()) 
    : 0)
    ;
     if (getPosRadians() <= LowerBoundRadians && speed < 0 || getPosRadians() >= UpperBoundRadians && speed > 0){ //Cuts motor if it hits its bounds and is trying to move past them
       motorStop();
      }
      SmartDashboard.putNumber("Extender:Input Speed", volts); //Lets us read the input value 
      m_extenderMotor.setControl(m_voltageOut.withOutput(volts));
  }

  public void holdPosition (double rotorRotations){ //Gets the motor back to where it shoudl be (Prevents overshoting). Basically a simple PID algorithm
    m_extenderMotor.setControl(m_PositionVoltage.withPosition(rotorRotations).withFeedForward(kG * Math.cos(getPosRadians())));
  }

  public final SingleJointedArmSim m_ArmSim = new SingleJointedArmSim( //Create a Physics Simulation 
    DCMotor.getKrakenX60(1),
    GearRatio,
    SingleJointedArmSim.estimateMOI(ExtenderLength, TotalExtenderWeight), //Moment Of Inertia Numbers
    ExtenderLength, //Arm Length
    LowerBoundRadians,
    UpperBoundRadians,
    true, //Gravity
    0.0); //Starting Angle In Radians

  @Override
  public void simulationPeriodic(){ //Applies Simulation every .update() seconds
    TalonFXSimState simState = m_extenderMotor.getSimState(); //Creates a "fake" TalonFX for the simulation

    simState.setSupplyVoltage(RobotController.getBatteryVoltage()); //Says "You have this much voltage available"
    m_ArmSim.setInputVoltage(m_extenderMotor.getSimState().getMotorVoltage()); //Says "Use this much voltage"

    m_ArmSim.update(0.020); //Advances Physics Model every 20 ms

    simState.setRawRotorPosition(Units.radiansToRotations(m_ArmSim.getAngleRads()) * GearRatio); //Sends Angle back to Motors TalonFX (Mainly to be used by getAngle)
    simState.setRotorVelocity(Units.radiansToRotations(m_ArmSim.getVelocityRadPerSec()) * GearRatio); //Sends Velocity back to Motors TalonFX
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extender:Current Position", getPosRadians());
    SmartDashboard.putNumber("Extender:Initial Position", initialPosRotations);
    SmartDashboard.putNumber("Extender:Voltage", m_extenderMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Extender:Current", m_extenderMotor.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Extender:Sim Voltage", m_extenderMotor.getSimState().getMotorVoltage());
    SmartDashboard.putData("Extender Mechanism", armMechanism2d);
    armLigament.setAngle(Math.toDegrees(getPosRadians()));
    }
    public Command setVoltageCommand(DoubleSupplier speed){
      return runEnd(() -> motorSetVoltage(speed.getAsDouble()), this::motorStop); //Runs motor untill its no longer being triggered. then it stops it.
    }
}
