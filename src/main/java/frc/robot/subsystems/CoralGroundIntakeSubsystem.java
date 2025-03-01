// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.GroundIntakeConstants;

public class CoralGroundIntakeSubsystem extends SubsystemBase {
  /** Creates a new GroundIntakeSubsystem. */
  private TalonFXConfiguration groundIntakeRollersConfig;
  private TalonFXConfiguration groundIntakePivotConfig;

  private TalonFX groundIntakeRollers;
  private TalonFX groundIntakePivot;
  
  public CoralGroundIntakeSubsystem() {
    groundIntakeRollers = new TalonFX(GroundIntakeConstants.kGroundIntakeMotorID);
    groundIntakePivot = new TalonFX(GroundIntakeConstants.kGroundIntakePivotID);


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
                                            .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANdiPWM2)
                                            .withSensorToMechanismRatio(GroundIntakeConstants.kSensorToMechanismRatio)
                                            .withRotorToSensorRatio(GroundIntakeConstants.kRotorToSensorRatio));

      groundIntakePivot.getConfigurator().apply(groundIntakePivotConfig);

      
  }

  public void groundIntakeRollers() {
    groundIntakeRollers.set(GroundIntakeConstants.kGroundIntakeSpeed);
  }

  public void groundIntakeRollersOff() {
    groundIntakeRollers.set(0);
  }

  public void groundOuttakeRollers() {
    groundIntakeRollers.set(GroundIntakeConstants.kGroundOuttakeSpeed);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
