// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.SetpointConstants;

public class CoralGroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new GroundIntakeSubsystem. */
  private TalonFXConfiguration groundIntakeRollersConfig;
  private TalonFXConfiguration groundIntakePivotConfig;

  private TalonFX groundIntakeRollers;
  private TalonFX groundIntakePivot;

  private MotionMagicVoltage m_motionRequest;

  private DigitalInput m_intakeBeamBreak;
  private DigitalInput m_hopperBeamBreak;

  
  public CoralGroundIntakeSubsystem() {
    groundIntakeRollers = new TalonFX(GroundIntakeConstants.kGroundIntakeMotorID);
    groundIntakePivot = new TalonFX(GroundIntakeConstants.kGroundIntakePivotID);

    m_intakeBeamBreak = new DigitalInput(GroundIntakeConstants.kGroundIntakeBeamBreakPort);
    m_hopperBeamBreak = new DigitalInput(GroundIntakeConstants.kHopperBeamBreakPort);


      groundIntakeRollersConfig = new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                          .withNeutralMode(NeutralModeValue.Brake));
      groundIntakeRollers.getConfigurator().apply(groundIntakeRollersConfig);

      groundIntakePivotConfig = new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                          .withNeutralMode(NeutralModeValue.Brake))
                            .withFeedback(new FeedbackConfigs()
                                            .withFeedbackRemoteSensorID(EndEffectorConstants.kCANdiID)
                                            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM2))
                            .withMotionMagic(new MotionMagicConfigs()
                                            .withMotionMagicCruiseVelocity(GroundIntakeConstants.kGroundIntakePivotMotionMagicCruiseVelocity)
                                            .withMotionMagicAcceleration(GroundIntakeConstants.kGroundIntakePivotMotionMagicAcceleration)
                                            .withMotionMagicJerk(GroundIntakeConstants.kGroundIntakePivotMotionMagicJerk))
                            .withSlot0(new Slot0Configs()
                                            .withKP(GroundIntakeConstants.kGroundPivotPIDValueP)
                                            .withKI(GroundIntakeConstants.kGroundPivotPIDValueI)
                                            .withKD(GroundIntakeConstants.kGroundPivotPIDValueD)
                                            .withGravityType(GravityTypeValue.Arm_Cosine));


      groundIntakePivot.getConfigurator().apply(groundIntakePivotConfig);

      m_motionRequest = new MotionMagicVoltage(0).withSlot(0).withFeedForward(GroundIntakeConstants.kGroundIntakePivotFeedForward);

      
  }

  public void groundRollersIntake() {
    groundIntakeRollers.set(-GroundIntakeConstants.kGroundRollersSpeed);
  }

  public void groundRollersStop() {
    groundIntakeRollers.set(0);
  }

  public void groundRollersOuttake() {
    groundIntakeRollers.set(GroundIntakeConstants.kGroundRollersSpeed);
  }

  public void groundIntakePivotUp() {
    groundIntakePivot.set(GroundIntakeConstants.kGroundPivotSpeed);
  }

  public void groundIntakePivotDown() {
    groundIntakePivot.set(-GroundIntakeConstants.kGroundPivotSpeed);
  }

  public void groundIntakePivotStop() {
    groundIntakePivot.set(0);
  }

  public void setPivotPosition(double Position){
    groundIntakePivot.setControl(m_motionRequest.withPosition(Position));
  }

  public BooleanSupplier isAtSetpoint(){
      return () -> Math.abs(getPivotPosition() - getPivotSetpoint()) < SetpointConstants.kSetpointThreshold;
    }
  
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Intake/IntakeBeamBreak")
  public boolean getIntakeSensor(){
    if(m_intakeBeamBreak.get()){
      return false;
    }else
    return true;
  }
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Hopper/HopperBeamBreak")
  public boolean getHopperSensor(){
    if(m_hopperBeamBreak.get()){
      return false;
    }else
    return true;
  }

  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/PivotPosition")
  public double getPivotPosition(){
    return groundIntakePivot.getPosition().getValueAsDouble();
  }

  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/PivotSetpoint")
  public double getPivotSetpoint(){
    return m_motionRequest.Position;
  }

  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/PivotVoltage")
  public double getPivotMotorVoltage(){
    return groundIntakePivot.getMotorVoltage().getValueAsDouble();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
