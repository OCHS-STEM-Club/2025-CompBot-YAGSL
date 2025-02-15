// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  private double lastDesiredPosition;
  // Top Limit
  private DigitalInput elevatorTopLimit;
  // Bottom Limit
  private DigitalInput elevatorBottonLimit;

  private VoltageOut m_voltageRequest;


  public ElevatorSubsystem() {
    // Elevator Motors
    elevatorLeftLeaderMotor = new TalonFX(ElevatorConstants.kElevatorLeftMotorID);
    elevatorRightFollowerMotor = new TalonFX(ElevatorConstants.kElevatorRightMotorID);
    // Elevator Follower
    elevatorFollower = new Follower(ElevatorConstants.kElevatorLeftMotorID, false);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
    // Last Desired Position
    lastDesiredPosition = 0;

    // Set Elevator Top Limit
    elevatorTopLimit = new DigitalInput(ElevatorConstants.kTopElevatorLimitPort);
    // Set Elevator Bottom Limit
    elevatorBottonLimit = new DigitalInput(ElevatorConstants.kBottomElevatorLimitPort);
    

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
                                        // .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                                        .withGravityType(GravityTypeValue.Elevator_Static))
                          // .withFeedback(new FeedbackConfigs()
                          //                   .withSensorToMechanismRatio(ElevatorConstants.kElevatorSensorToMechRatio))//Need to get sensor to mechanism ratio
                          .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.Clockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake))
                          .withMotionMagic(new MotionMagicConfigs()
                                              .withMotionMagicCruiseVelocity(ElevatorConstants.kElevatorMotionMagicCruiseVelocity)
                                              .withMotionMagicAcceleration(ElevatorConstants.kElevatorMotionMagicAcceleration)
                                              .withMotionMagicJerk(ElevatorConstants.kElevatorMotionMagicJerk));

                          // .withCurrentLimits(new CurrentLimitsConfigs()
                          //                     .withStatorCurrentLimit(40));
                                              
    // Apply elevatorConfigs
    elevatorLeftLeaderMotor.getConfigurator().apply(elevatorConfigs);
    elevatorRightFollowerMotor.getConfigurator().apply(elevatorConfigs);

    


    // Elevator Position Request
    elevatorPositionRequest = new MotionMagicVoltage(0).withSlot(0);

    m_voltageRequest = new VoltageOut(0.0);

    

    }

  // Elevator Up
  public void elevatorUp() {  
    elevatorLeftLeaderMotor.setVoltage(1.2);;
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }
  // Elevator Down
  public void elevatorDown() {
      elevatorLeftLeaderMotor.setVoltage(-1.2);
      elevatorRightFollowerMotor.setControl(elevatorFollower);
    
  }
  // Elevator Stop
  public void elevatorStop() {
    elevatorLeftLeaderMotor.set(0);;;
    elevatorRightFollowerMotor.setControl(elevatorFollower);
  }

  // set Elevator Position
  public void setElevatorPosition(double height) {
    elevatorLeftLeaderMotor.setControl(elevatorPositionRequest.withPosition(height)
                                      .withLimitForwardMotion(isAtTopLimit())
                                      .withLimitReverseMotion(isAtBottomLimit()));
    // elevatorRightFollowerMotor.setControl(elevatorFollower);
    lastDesiredPosition = height;

  }

  public boolean isBelow10Inches(){
    return this.getElevatorPositionInches() < 10;
  }

  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorRequestPosition")
  public double getElevatorRequestPosition(){
    return elevatorPositionRequest.Position;
  }




private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(0.5).per(Second),        // Use default ramp rate (1 V/s)
         Volts.of(1.2), // Reduce dynamic step voltage to 4 to prevent brownout
         Seconds.of(5.3),        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> elevatorLeftLeaderMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return m_sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return m_sysIdRoutine.dynamic(direction);
}
  
  // get Elevator Position
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorPositionRotations")
  public double getElevatorPositionInches() {
    return elevatorLeftLeaderMotor.getRotorPosition().getValueAsDouble();
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

  // get Elevator Current
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorCurrent")
  public double getElevatorCurrent(){
    return elevatorLeftLeaderMotor.getStatorCurrent().getValueAsDouble();
  }

  // get Elevator Voltage
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorVoltage")
  public double getElevatorVoltage(){
    return elevatorLeftLeaderMotor.getMotorVoltage().getValueAsDouble();
  }

  // get Elevator last Desired Position
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorLastDesiredPosition")
  public double  getLastDesiredPosition() {
    return lastDesiredPosition;
  }

  // is at Setpoint?
  // @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorIsAtSetpoint?")
  // public boolean isAtSetpoint() {
  //   return ((getElevatorPositionDouble() == (getLastDesiredPosition() - (ElevatorConstants.kElevatorSetpointThreshold))) > 0 )&&(
  //         (getElevatorPositionDouble() == (getLastDesiredPosition() + (ElevatorConstants.kElevatorSetpointThreshold))) < 0);
  // }
  
  // is at Bottom Limit?
  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorIsAtTopLimit?")
  public boolean isAtTopLimit() {
    if(elevatorTopLimit.get()){
      return false;
    }else{
      return true;
    }
  }

  @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/ElevatorIsAtBottomLimit?")
  public boolean isAtBottomLimit(){
    if(elevatorBottonLimit.get()){
      return false;
    }else{
      return true;
    }
  }

  @Override
  public void periodic() {

    if(isAtBottomLimit()){
      elevatorLeftLeaderMotor.setPosition(0);
    }
   
  }

}
