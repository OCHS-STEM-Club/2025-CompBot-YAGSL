// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  // Pivot PID Controller
  private PIDController pivotPIDController;

  // Voltage Request
  private VoltageOut m_voltageRequest;

  private CANdi canDi;

  private CANdiConfiguration canDiConfigs;

  private PositionVoltage m_positionRequest;

  public EndEffectorSubsystem() {
    // End Effector Intake
    endEffectorIntake = new TalonFXS(EndEffectorConstants.kEndEffectorIntakeID);
    // End Effector Pivot
    endEffectorPivot = new TalonFX(EndEffectorConstants.kEndEffectorPivotID);

    // Intake Beam Break
    intakeBeamBreak = new DigitalInput(EndEffectorConstants.kEndEffectorBeamBreakPort);

    // CANdi 
    canDi = new CANdi(EndEffectorConstants.kCANdiID);

    // CANdi Configs
    canDiConfigs = new CANdiConfiguration()
                        .withPWM1(new PWM1Configs()
                                      .withAbsoluteSensorOffset(EndEffectorConstants.kPWM1AbsoluteEncoderOffset)
                                      .withAbsoluteSensorDiscontinuityPoint(EndEffectorConstants.kPWM1AbsoluteEncoderDiscontinuityPoint));

    canDi.getConfigurator().apply(canDiConfigs);

    // Intake Configs
    intakeConfigs = new TalonFXSConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive)
                                              .withNeutralMode(NeutralModeValue.Brake))
                        .withCommutation(new CommutationConfigs()
                                              .withMotorArrangement(MotorArrangementValue.Minion_JST));

    // Apply Intake Configs
    endEffectorIntake.getConfigurator().apply(intakeConfigs);

    // Pivot Configs
    pivotConfigs = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake))
                        .withFeedback(new FeedbackConfigs()
                                            .withFeedbackRemoteSensorID(EndEffectorConstants.kCANdiID)
                                            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM1));
    // Apply Pivot Configs
    endEffectorPivot.getConfigurator().apply(pivotConfigs);
  
    // SysID voltage request
    m_voltageRequest = new VoltageOut(0);

    m_positionRequest = new PositionVoltage(0).withSlot(0);



  }


  // End Effector Intake
  public void rollersIntake() {
    endEffectorIntake.set(EndEffectorConstants.kEndEffectorSpeed);
  }

  // End Effector Outtake
  public void rollersOuttake() {
    endEffectorIntake.set(-EndEffectorConstants.kEndEffectorSpeed);
  }

  // End Effector Stop
  public void rollersStop() {
    endEffectorIntake.set(0);
  }

  // Intake with current spike detection
  public void intakeCoralWithCurrentSpikeDetection() {
    if (endEffectorIntake.getSupplyCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike) {
      endEffectorIntake.set(0);
    } else {
      endEffectorIntake.set(EndEffectorConstants.kEndEffectorSpeed);
    }
  }

  // Intake with beam break detection
  public void intakeAlgaeWithBeamBreak() {
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


  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
          null,         // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          Seconds.of(4),           // Use default timeout (10 s)
                                          // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("End Effector State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
          (volts) -> endEffectorPivot.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
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

  public void setPivotPosition(){
    // endEffectorPivot.setControl(voltageRequest.withpo)
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
    return endEffectorPivot.getStatorCurrent().getValueAsDouble();
  }

  // Get Intake Velocity
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeVelocity")
  public double getIntakeVelocity() {
    return endEffectorIntake.get();
  }

  // Get Intake Current
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeCurrent")
  public double getIntakeCurrent() {
    return endEffectorIntake.getTorqueCurrent().getValueAsDouble();
  }

  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/EndEffectorCANdiPWM1Position")
  public double getCANdiPosition(){
    return canDi.getPWM1Position().getValueAsDouble();
  }

  public CANdi getCANdi(){
    return canDi;
  }




  @Override
  public void periodic() {

  }
  





}


  





