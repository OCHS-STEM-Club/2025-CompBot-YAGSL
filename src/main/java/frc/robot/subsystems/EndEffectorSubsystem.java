// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new EndEffector. */

  // End Effector Intake
  private TalonFXS endEffectorIntake;
  // End Effector Pivot
  private TalonFX endEffectorPivot;

  // End Effector Configs
  private TalonFXSConfiguration intakeConfigs;
  // Pivot Configs
  private TalonFXConfiguration pivotConfigs;
  
  // Intake Beam Break
  private DigitalInput intakeBeamBreak;
  // Pivot Encoder
  private DutyCycleEncoder pivotEncoder;

  // Pivot PID Controller
  private PIDController pivotPIDController;

  public EndEffectorSubsystem() {
    // End Effector Intake
    endEffectorIntake = new TalonFXS(EndEffectorConstants.kEndEffectorIntakeID);
    // End Effector Pivot
    endEffectorPivot = new TalonFX(EndEffectorConstants.kEndEffectorPivotID);

    // Intake Beam Break
    intakeBeamBreak = new DigitalInput(EndEffectorConstants.kEndEffectorBeamBreakPort);

    // Pivot Encoder
    pivotEncoder = new DutyCycleEncoder(EndEffectorConstants.kEndEffectorPivotEncoderPort);

    // Intake Configs
    intakeConfigs = new TalonFXSConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake));

    // Apply Intake Configs
    endEffectorIntake.getConfigurator().apply(intakeConfigs);

    // Pivot Configs
    pivotConfigs = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake));
    // Apply Pivot Configs
    endEffectorPivot.getConfigurator().apply(pivotConfigs);
  
    // Pivot PID Controller
    pivotPIDController = new PIDController(EndEffectorConstants.kEndEffectorPivotPIDValueP, 
                                           EndEffectorConstants.kEndEffectorPivotPIDValueI, 
                                           EndEffectorConstants.kEndEffectorPivotPIDValueD);
  }


  // End Effector Intake
  public void endEffectorIntake() {
    endEffectorIntake.set(EndEffectorConstants.kEndEffectorSpeed);
  }

  // End Effector Outtake
  public void endEffectorOuttake() {
    endEffectorIntake.set(-EndEffectorConstants.kEndEffectorSpeed);
  }

  // End Effector Stop
  public void endEffectorStop() {
    endEffectorIntake.set(0);
  }

  // Intake with current spike detection
  public void intakeCoral() {
    if (endEffectorIntake.getSupplyCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike) {
      endEffectorIntake.set(0);
    } else {
      endEffectorIntake.set(EndEffectorConstants.kEndEffectorSpeed);
    }
  }

  // Intake with beam break detection
  public void intakeAlgae() {
    if (intakeBeamBreak.get()) {
      endEffectorIntake.set(0);
    } else {
      endEffectorIntake.set(EndEffectorConstants.kEndEffectorSpeed);
    }
  }

  // Do we have coral?
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/HasCoral?")
  public boolean hasCoral(){
    return endEffectorIntake.getSupplyCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike;
  }

  // Do we have algae?
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/HasAlgae?")
  public boolean hasAlgae(){
    return intakeBeamBreak.get();
  }


  // End Effector Pivot Up
  public void EndEffectorPivotUp() {
    endEffectorPivot.set(EndEffectorConstants.kPivotSpeed);
  }

  // End Effector Pivot Down
  public void EndEffectorPivotDown() {
    endEffectorPivot.set(-EndEffectorConstants.kPivotSpeed);
  }

  // End Effector Pivot Stop
  public void EndEffectorPivotStop() {
    endEffectorPivot.set(0);
  } 

  // End Effector Pivot Position Control
  public void setPivotPosition(double position) {
    endEffectorPivot.set(pivotPIDController.calculate(pivotEncoder.get(), position));
  }


  // Get Pivot Position
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotPosition")
  public double getPivotPosition() {
    return endEffectorPivot.getPosition().getValueAsDouble();
  }

  // Get Pivot Velocity
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotVelocity")
  public double getPivotVelocity() {
    return endEffectorPivot.get();
  }

  // Get Pivot Current
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotCurrent")
  public double getPivotCurrent() {
    return endEffectorPivot.getSupplyCurrent().getValueAsDouble();
  }

  // Get Intake Velocity
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeVelocity")
  public double getIntakeVelocity() {
    return endEffectorIntake.get();
  }

  // Get Intake Current
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeCurrent")
  public double getIntakeCurrent() {
    return endEffectorIntake.getSupplyCurrent().getValueAsDouble();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Subsystems/EndEffectorSubsystem/Intake/EndEffectorHasCoral?", hasCoral());
    SmartDashboard.putBoolean("Subsystems/EndEffectorSubsystem/Intake/EndEffectorHasAlgae?", hasAlgae());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotPosition", getPivotPosition());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotVelocity", getPivotVelocity());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotCurrent", getPivotCurrent());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeVelocity", getIntakeVelocity());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeCurrent", getIntakeCurrent());

  }
  





}


  





