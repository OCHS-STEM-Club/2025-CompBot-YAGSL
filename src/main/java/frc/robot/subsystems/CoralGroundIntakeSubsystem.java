// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.GroundIntakeConstants;

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

    m_intakeBeamBreak = new DigitalInput(8);
    m_hopperBeamBreak = new DigitalInput(9);


      groundIntakeRollersConfig = new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                          .withInverted(InvertedValue.Clockwise_Positive)
                                          .withNeutralMode(NeutralModeValue.Brake));
      groundIntakeRollers.getConfigurator().apply(groundIntakeRollersConfig);

      groundIntakePivotConfig = new TalonFXConfiguration()
                            .withMotorOutput(new MotorOutputConfigs()
                                          .withInverted(InvertedValue.Clockwise_Positive)
                                          .withNeutralMode(NeutralModeValue.Brake))
                            .withMotionMagic(new MotionMagicConfigs()
                                          .withMotionMagicCruiseVelocity(GroundIntakeConstants.kGroundIntakePivotMotionMagicCruiseVelocity)
                                          .withMotionMagicAcceleration(GroundIntakeConstants.kGroundIntakePivotMotionMagicCruiseAcceleration)
                                          .withMotionMagicJerk(GroundIntakeConstants.kGroundIntakePivotMotionMagicCruiseJerk))
                            .withFeedback(new FeedbackConfigs()
                                            .withFeedbackRemoteSensorID(EndEffectorConstants.kCANdiID)
                                            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM2))
                            .withSlot0(new Slot0Configs()
                                            .withKP(0.11239)
                                            .withKI(0)
                                            .withKD(0)
                                            .withGravityType(GravityTypeValue.Arm_Cosine))
                            .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicAcceleration(6425)
                                                .withMotionMagicCruiseVelocity(5461)
                                                .withMotionMagicJerk(0));

      groundIntakePivot.getConfigurator().apply(groundIntakePivotConfig);

      m_motionRequest = new MotionMagicVoltage(0).withSlot(0).withFeedForward(0.022094);

      
  }

  public void groundIntakeRollers() {
    groundIntakeRollers.set(GroundIntakeConstants.kGroundSpeed);
  }

  public void groundIntakeRollersOff() {
    groundIntakeRollers.set(0);
  }

  public void groundOuttakeRollers() {
    groundIntakeRollers.set(-GroundIntakeConstants.kGroundSpeed);
  }

  public void groundIntakePivotUp() {
    groundIntakePivot.set(GroundIntakeConstants.kGroundIntakePivotSpeed);
  }

  public void groundIntakePivotDown() {
    groundIntakePivot.set(-GroundIntakeConstants.kGroundIntakePivotSpeed);
  }

  public void groundIntakePivotStop() {
    groundIntakePivot.set(0);
  }

  public void setPivotPosition(double Position){
    groundIntakePivot.setControl(m_motionRequest.withPosition(Position));
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
  public double getIntakePosition(){
    return groundIntakePivot.getPosition().getValueAsDouble();
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
