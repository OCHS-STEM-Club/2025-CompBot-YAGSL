// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.SetpointConstants;

public class CoralGroundIntakeSubsystem extends SubsystemBase {

  // Motors
  private TalonFX groundIntakeRollers;
  private TalonFX groundIntakePivot;

  // Motor Configs
  private TalonFXConfiguration groundIntakeRollersConfig;
  private TalonFXConfiguration groundIntakePivotConfig;

  // Motion Request
  private MotionMagicVoltage m_motionRequest;

  // Sensors
  private DigitalInput m_intakeBeamBreak;
  private DigitalInput m_hopperBeamBreak;



  
  public CoralGroundIntakeSubsystem() {

    // Motors
    groundIntakeRollers = new TalonFX(GroundIntakeConstants.kGroundIntakeMotorID);
    groundIntakePivot = new TalonFX(GroundIntakeConstants.kGroundIntakePivotID);

    // Sensors
    m_intakeBeamBreak = new DigitalInput(GroundIntakeConstants.kGroundIntakeBeamBreakPort);
    m_hopperBeamBreak = new DigitalInput(GroundIntakeConstants.kHopperBeamBreakPort);


    // Rollers Config
    groundIntakeRollersConfig = new TalonFXConfiguration()
                          .withMotorOutput(new MotorOutputConfigs()
                                        .withInverted(InvertedValue.Clockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake));
                                        
    groundIntakeRollers.getConfigurator().apply(groundIntakeRollersConfig);

    // Pivot Configs
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
                                          .withGravityType(GravityTypeValue.Arm_Cosine))
                          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                                            .withContinuousWrap(true));;

    groundIntakePivot.getConfigurator().apply(groundIntakePivotConfig);

    // Motion Request
    m_motionRequest = new MotionMagicVoltage(0).withSlot(0).withFeedForward(GroundIntakeConstants.kGroundIntakePivotFeedForward);
  }

  // Roller Intake
  public void groundRollersIntake() {
    groundIntakeRollers.set(-GroundIntakeConstants.kGroundRollersSpeedIntakeSpeed);
  }

  public void groundRollersIntakeSpeed(double speed) {
    groundIntakeRollers.set(-speed);
  }
  // Rollers Outtake
  public void groundRollersOuttake() {
    groundIntakeRollers.set(GroundIntakeConstants.kGroundRollersSpeedOuttakeSpeed);
  }
  // Rollers Stop
  public void groundRollersStop() {
    groundIntakeRollers.set(0);
  }

  // Pivot Up
  public void groundIntakePivotUp() {
    groundIntakePivot.set(GroundIntakeConstants.kGroundPivotSpeed);
  }
  // Pivot Down
  public void groundIntakePivotDown() {
    groundIntakePivot.set(-GroundIntakeConstants.kGroundPivotSpeed);
  }
  // Pivot Stop
  public void groundIntakePivotStop() {
    groundIntakePivot.set(0);
  }

  // Intake coral with Intake Sensor
  public Command intakeWithSensor(){
    return Commands.run(()-> {
      if(getIntakeSensor() == false){
        groundRollersIntake();
      }else
        groundRollersStop();
      
    });
  }

  public BooleanSupplier getHopperSensorSupplier(){
    return () -> getHopperSensor();
  }

  public BooleanSupplier getIntakeSensorSupplier(){
    return () -> getIntakeSensor();
  }

  // Set Pivot Position
  public void setPivotPosition(double Position){
    groundIntakePivot.setControl(m_motionRequest.withPosition(Position));
  }

  // isAtSetpoint?
  @AutoLogOutput(key = "Subsystems/CoralGroundIntakeSubsystem/Pivot/Position/IsAtSetpoint?")
  public BooleanSupplier isAtSetpoint(){
      return () -> Math.abs(getPivotPosition() - getPivotSetpoint()) < 0.05;
    }

    public BooleanSupplier isAtStowSetpoint(){
      return () -> Math.abs(getPivotPosition() - 0.65) < 0.05;
    }
  
  // Get Pivot Position
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/Position/PivotPosition")
  public double getPivotPosition(){
    return groundIntakePivot.getPosition().getValueAsDouble();
  }

  // Get Pivot Setpoint
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/Position/PivotSetpoint")
  public double getPivotSetpoint(){
    return m_motionRequest.Position;
  }

  // get Pivot Velocity
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/Motor/PivotVelocity")
  public double getPivotVelocity(){
    return groundIntakePivot.get();
  }

  // get Pivot Current
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/Motor/PivotCurrent")
  public double getPivotCurrent(){
    return groundIntakePivot.getSupplyCurrent().getValueAsDouble();
  }

  // get Pivot Voltage
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/Motor/PivotVoltage")
  public double getPivotMotorVoltage(){
    return groundIntakePivot.getMotorVoltage().getValueAsDouble();
  }

  // // get Pivot Temp
  // @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Pivot/Motor/PivotTemperature")
  // public double getPivotMotorTemp(){
  //   return groundIntakePivot.getDeviceTemp().getValueAsDouble();
  // }

  // get Intake Velocity
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Intake/Motor/IntakeVelocity")
  public double getIntakeVelocity(){
    return groundIntakeRollers.get();
  }

  // get Intake Current
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Intake/Motor/IntakeCurrent")
  public double getIntakeCurrent(){
    return groundIntakeRollers.getSupplyCurrent().getValueAsDouble();
  }

  // get Intake Voltage
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Intake/Motor/IntakeVoltage")
  public double getIntakeMotorVoltage(){
    return groundIntakeRollers.getMotorVoltage().getValueAsDouble();
  }

  // // get Intake Temp
  // @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Intake/Motor/IntakeTemperature")
  // public double getIntakeMotorTemp(){
  //   return groundIntakeRollers.getDeviceTemp().getValueAsDouble();
  // }






  // Get Intake Sensor
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Intake/IntakeSensor")
  public boolean getIntakeSensor(){
    if(m_intakeBeamBreak.get()){
      return false;
    }else
    return true;
  }

  // Get Hopper Sensor
  @AutoLogOutput(key  = "Subsystems/CoralGroundIntakeSubsystem/Hopper/HopperSensor")
  public boolean getHopperSensor(){
    if(m_hopperBeamBreak.get()){
      return false;
    }else
    return true;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

  
  }

}


