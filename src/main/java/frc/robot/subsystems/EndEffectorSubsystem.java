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
  // private DigitalInput intakeBeamBreak;
  // Pivot Encoder
  private DutyCycleEncoder pivotEncoder;

  // Pivot PID Controller
  private PIDController pivotPIDController;

  private VoltageOut voltageRequest;

  private DigitalInput input;

  private CANdi canDi;

  private CANdiConfiguration canDiConfigs;

  public EndEffectorSubsystem() {
    // End Effector Intake
    endEffectorIntake = new TalonFXS(EndEffectorConstants.kEndEffectorIntakeID);
    // End Effector Pivot
    endEffectorPivot = new TalonFX(EndEffectorConstants.kEndEffectorPivotID);

    input  = new DigitalInput(3);
    // Intake Beam Break
    // intakeBeamBreak = new DigitalInput(EndEffectorConstants.kEndEffectorBeamBreakPort);

    // Pivot Encoder
    pivotEncoder = new DutyCycleEncoder(input);

    canDi = new CANdi(26);

    canDiConfigs = new CANdiConfiguration()
                        .withPWM1(new PWM1Configs()
                                      .withAbsoluteSensorOffset(-0.2553)
                                      .withAbsoluteSensorDiscontinuityPoint(0.5));

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
                                            .withFeedbackRemoteSensorID(26)
                                            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM1)
                                            .withFeedbackRotorOffset(0.2));
    // Apply Pivot Configs
    endEffectorPivot.getConfigurator().apply(pivotConfigs);
  
    // Pivot PID Controller
    pivotPIDController = new PIDController(EndEffectorConstants.kEndEffectorPivotPIDValueP, 
                                           EndEffectorConstants.kEndEffectorPivotPIDValueI, 
                                           EndEffectorConstants.kEndEffectorPivotPIDValueD);
    // Sys id voltage request
    voltageRequest = new VoltageOut(0);



    // pivotEncoder.setDutyCycleRange(0, 1);
    // pivotEncoder.setAssumedFrequency(0);
    // pivotEncoder.setDutyCycleRange(1/1024,1023/1024);
    // pivotEncoder.setAssumedFrequency(975.6);
    // pivotEncoder.setConnectedFrequencyThreshold(1);
    // pivotEncoder.setInverted(true);

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
  // public void intakeAlgaeWithBeamBreak() {
  //   if (intakeBeamBreak.get()) {
  //     endEffectorIntake.set(0);
  //   } else {
  //     endEffectorIntake.set(EndEffectorConstants.kEndEffectorSpeed);
  //   }
  // }

  // Do we have coral?
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/HasCoral?")
  public boolean hasCoral(){
    return endEffectorIntake.getSupplyCurrent().getValueAsDouble() > EndEffectorConstants.kEndEffectorCurrentSpike;
  }

  // Do we have algae?
  // @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Intake/HasAlgae?")
  // public boolean hasAlgae(){
  //   return intakeBeamBreak.get();
  // }


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
  public void setEndEffectorPivotPosition(double position) {
    endEffectorPivot.set(pivotPIDController.calculate(pivotEncoder.get(), position));
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
          (volts) -> endEffectorPivot.setControl(voltageRequest.withOutput(volts.in(Volts))),
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

  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotEncoder")
  public double getPivotEncoder() {
    return pivotEncoder.get() * (2 * Math.PI);
  }
  // Get Pivot Encoder
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/IsPivotEncoderConnected?")
  public boolean isPivotEncoderConnected() {
    return pivotEncoder.isConnected();
  }

  
  // Get Pivot Encoder Frequency
  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotEncoderFrequency")
  public int getPivotEncoderFrequency() {
    return pivotEncoder.getFrequency();
  }

  @AutoLogOutput(key = "Subsystems/EndEffectorSubsystem/Pivot/EndEffectorCANdiPWM1Position")
  public double getCANdiPosition(){
    return canDi.getPWM1Position().getValueAsDouble();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Subsystems/EndEffectorSubsystem/Intake/EndEffectorHasCoral?", hasCoral());
    // SmartDashboard.putBoolean("Subsystems/EndEffectorSubsystem/Intake/EndEffectorHasAlgae?", hasAlgae());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotPosition", getPivotPosition());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotVelocity", getPivotVelocity());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotCurrent", getPivotCurrent());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeVelocity", getIntakeVelocity());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Intake/EndEffectorIntakeCurrent", getIntakeCurrent());
    SmartDashboard.putNumber("Subsystems/EndEffectorSubsystem/Pivot/EndEffectorPivotEncoderPosition", getPivotEncoder());

  }
  





}


  





