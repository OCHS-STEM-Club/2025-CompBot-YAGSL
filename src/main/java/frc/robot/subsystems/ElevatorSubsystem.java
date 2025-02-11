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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // Elevator Motors
  private TalonFX elevatorLeftLeaderMotor;
  private TalonFX elevatorRightFollowerMotor;
  // Elevator Configs
  private TalonFXConfiguration elevatorConfigs;
  // Elevator Follower
  private Follower elevatorFollower;
  // Elevator Position Request
  private MotionMagicVoltage elevatorPositionRequest;
  // Last Desired Position
  private Distance lastDesiredPosition;

  public ElevatorSubsystem() {
    // Elevator Motors
    elevatorLeftLeaderMotor = new TalonFX(ElevatorConstants.kElevatorLeftMotorID);
    elevatorRightFollowerMotor = new TalonFX(ElevatorConstants.kElevatorRightMotorID);
    // Elevator Follower
    elevatorFollower = new Follower(ElevatorConstants.kElevatorLeftMotorID, false);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
    // Elevator Position Request
    elevatorPositionRequest = new MotionMagicVoltage(0);
    // Last Desired Position
    lastDesiredPosition = Units.Inches.of(0);

    // elevatorConfigs
    elevatorConfigs = new TalonFXConfiguration()
                          .withSlot0(new Slot0Configs()
                                        .withKP(ElevatorConstants.kElevatorPIDValueP)
                                        .withKI(ElevatorConstants.kElevatorPIDValueI)
                                        .withKD(ElevatorConstants.kElevatorPIDValueD)
                                        .withKS(ElevatorConstants.kElevatorPIDValueS)
                                        .withKV(ElevatorConstants.kElevatorPIDValueV)
                                        .withKA(ElevatorConstants.kElevatorPIDValueA)
                                        .withKG(ElevatorConstants.kElevatorPIDValueG)
                                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withGravityType(GravityTypeValue.Elevator_Static))
                          .withFeedback(new FeedbackConfigs()
                                            .withSensorToMechanismRatio(ElevatorConstants.kElevatorSensorToMechRatio))//Need to get sensor to mechanism ratio
                          .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.Clockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake))
                          .withMotionMagic(new MotionMagicConfigs()
                                              .withMotionMagicCruiseVelocity(ElevatorConstants.kElevatorMotionMagicCruiseVelocity)
                                              .withMotionMagicAcceleration(ElevatorConstants.kElevatorMotionMagicAcceleration)); 
    // Apply elevatorConfigs
    elevatorLeftLeaderMotor.getConfigurator().apply(elevatorConfigs);
    elevatorRightFollowerMotor.getConfigurator().apply(elevatorConfigs);


    }

  // Elevator Up
  public void elevatorUp() {  
    elevatorLeftLeaderMotor.set(ElevatorConstants.kElevatorSpeed);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }
  // Elevator Down
  public void elevatorDown() {
    elevatorLeftLeaderMotor.set(-ElevatorConstants.kElevatorSpeed);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }
  // Elevator Stop
  public void elevatorStop() {
    elevatorLeftLeaderMotor.set(0);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }

  // set Elevator Position
  public void setElevatorPosition(Distance height) {
    elevatorLeftLeaderMotor.setControl(elevatorPositionRequest.withPosition(height.in(Units.Inches)));
    elevatorRightFollowerMotor.setControl(elevatorFollower);
    lastDesiredPosition = height;

  }


  // get Elevator Position
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorPosition")
  public Distance getElevatorPosition() {
    return Units.Inches.of(elevatorLeftLeaderMotor.getPosition().getValueAsDouble());
  }
  public double getElevatorPositionDouble() {
    return elevatorLeftLeaderMotor.getPosition().getValueAsDouble();
  }
  // get Elevator Left Motor Velocity
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorLeftMotorVelocity")
  public double getElevatorLeftMotorVelocity() {
    return elevatorLeftLeaderMotor.get();
  }
  // get Elevator Right Motor Velocity
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorRightMotorVelocity")
  public double getElevatorRightMotorVelocity() {
    return elevatorRightFollowerMotor.get();
  }
  // get Elevator last Desired Position
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorLastDesiredPosition")
  public Distance getLastDesiredPosition() {
    return lastDesiredPosition;
  }
  // is at Setpoint?
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorIsAtSetpoint?")
  public boolean isAtSetpoint() {
    return (getElevatorPosition().compareTo(getLastDesiredPosition().minus(Units.Inches.of(0.5))) > 0) &&
            getElevatorPosition().compareTo(getLastDesiredPosition().plus(Units.Inches.of(0.5))) < 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Subsystems/ElevatorSubsystem/Elevator/ElevatorIsAtSetpoint?", isAtSetpoint());
    SmartDashboard.putNumber("Subsystems/ElevatorSubsystem/Elevator/ElevatorPosition", getElevatorPositionDouble());
    SmartDashboard.putNumber("Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorLeftMotorVelocity", getElevatorLeftMotorVelocity());
    SmartDashboard.putNumber("Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorRightMotorVelocity", getElevatorRightMotorVelocity());
  }

}
