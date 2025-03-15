// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SpeedConstants;

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
  // Bottom Limit
  private DigitalInput elevatorBottonLimit;
  // Voltage Request
  private VoltageOut m_voltageRequest;


  public ElevatorSubsystem() {
    // Elevator Motors
    elevatorLeftLeaderMotor = new TalonFX(ElevatorConstants.kElevatorLeftMotorID);
    elevatorRightFollowerMotor = new TalonFX(ElevatorConstants.kElevatorRightMotorID);
    // Elevator Follower
    elevatorFollower = new Follower(ElevatorConstants.kElevatorLeftMotorID, false);
    elevatorRightFollowerMotor.setControl(elevatorFollower);
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
                                        .withGravityType(GravityTypeValue.Elevator_Static))
                          .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.Clockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake))
                          .withMotionMagic(new MotionMagicConfigs()
                                              .withMotionMagicCruiseVelocity(ElevatorConstants.kElevatorMotionMagicCruiseVelocity)
                                              .withMotionMagicAcceleration(ElevatorConstants.kElevatorMotionMagicAcceleration)
                                              .withMotionMagicJerk(ElevatorConstants.kElevatorMotionMagicJerk));

    // Apply elevatorConfigs
    elevatorLeftLeaderMotor.getConfigurator().apply(elevatorConfigs);
    elevatorRightFollowerMotor.getConfigurator().apply(elevatorConfigs);

    // Elevator Position Request
    elevatorPositionRequest = new MotionMagicVoltage(0).withSlot(0);
    // Voltage Request
    m_voltageRequest = new VoltageOut(0.0);


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
    public void setElevatorPosition(double height) {
      elevatorLeftLeaderMotor.setControl(elevatorPositionRequest.withPosition(height)
          .withLimitReverseMotion(isAtBottomLimit()));
      elevatorRightFollowerMotor.setControl(elevatorFollower);
    }

    // Defines SysID Configs
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), // Use default ramp rate (1 V/s)
            Volts.of(1.2), // Reduce dynamic step voltage to 4 to prevent brownout
            Seconds.of(5.3), // Use default timeout (10 s)
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("Elevator state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> elevatorLeftLeaderMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
            null,
            this));
    
    // SysID Quasistatic Command
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.quasistatic(direction);
    }

    // SysID Dynamic Command
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
    }

    // is at Setpoint?
    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/Position/IsAtSetpoint?")
    public BooleanSupplier isAtSetpoint(){
      return () -> Math.abs(getElevatorPositionRotations() - getElevatorPositionSetpoint()) < SetpointConstants.kSetpointThreshold;
    }

    // get Elevator Position
    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/Position/ElevatorPosition")
    public double getElevatorPositionRotations() {
      return elevatorLeftLeaderMotor.getRotorPosition().getValueAsDouble();
    }

    // get Elevator Setpoint
    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/Elevator/Position/ElevatorSetpoint")
    public double getElevatorPositionSetpoint() {
      return elevatorPositionRequest.Position;
    }

    // get Elevator Left Motor Velocity
    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorVelocity")
    public double getElevatorVelocity() {
      return elevatorLeftLeaderMotor.get();
    }

    // get Elevator Current
    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorCurrent")
    public double getElevatorCurrent() {
      return elevatorLeftLeaderMotor.getSupplyCurrent().getValueAsDouble();
    }

    // get Elevator Voltage
    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorVoltage")
    public double getElevatorVoltage() {
      return elevatorLeftLeaderMotor.getMotorVoltage().getValueAsDouble();
    }

    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorMotors/ElevatorTemperature")
    public double getElevatorMotorTemp(){
      return elevatorLeftLeaderMotor.getDeviceTemp().getValueAsDouble();
    }

    // is at Bottom Limit?
    @AutoLogOutput(key = "Subsystems/ElevatorSubsystem/ElevatorLimits/ElevatorIsAtBottomLimit?")
    public boolean isAtBottomLimit() {
      if (elevatorBottonLimit.get()) {
        return false;
      } else {
        return true;
      }
    }

    public Command reduceRobotSpeed() {
      return Commands.run(() -> {
        SpeedConstants.kCurrentRobotTranslationSpeed = SpeedConstants.kReducedRobotTranslationSpeed;
        SpeedConstants.kCurrentRobotRotationSpeed = SpeedConstants.kReducedRobotRotationSpeed;
      });
    }

    public Command normalRobotSpeed() {
      return Commands.runOnce(() -> {
        SpeedConstants.kCurrentRobotTranslationSpeed = SpeedConstants.kNormalRobotTranslationSpeed;
        SpeedConstants.kCurrentRobotRotationSpeed = SpeedConstants.kNormalRobotRotationSpeed;
      });
    }

    @Override
    public void periodic() {
      // set Elevator Zero Position
      if (isAtBottomLimit()) {
        elevatorLeftLeaderMotor.setPosition(0);
      }


    }

}
