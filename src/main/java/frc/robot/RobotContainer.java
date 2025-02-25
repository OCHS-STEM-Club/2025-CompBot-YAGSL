// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.Manual.Elevator.ElevatorManualDown;
import frc.robot.commands.Manual.Elevator.ElevatorManualUp;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualIntake;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualOuttake;
import frc.robot.commands.Manual.EndEffector.Pivot.EndEffectorManualPivotDown;
import frc.robot.commands.Manual.EndEffector.Pivot.EndEffectorManualPivotUp;
import frc.robot.commands.Sequential.CS_CMD;
import frc.robot.commands.Sequential.L1_CMD;
import frc.robot.commands.Sequential.L2_CMD;
import frc.robot.commands.Sequential.L3_CMD;
import frc.robot.commands.Sequential.STOW_CMD;
import frc.robot.commands.Setpoints_CMD.Elevator_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SendableChooser<Command> autoChooser;

  // Controller definitions
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandJoystick m_operatorController1 = new CommandJoystick(1);
  CommandJoystick m_operatorController2 = new CommandJoystick(2);


  private final Trigger DRIVER_A_BUTTON = new Trigger(() -> m_driverController.getHID().getAButton());
  private final Trigger DRIVER_B_BUTTON = new Trigger(() -> m_driverController.getHID().getBButton());
  private final Trigger DRIVER_X_BUTTON = new Trigger(() -> m_driverController.getHID().getXButton());
  private final Trigger DRIVER_Y_BUTTON = new Trigger(() -> m_driverController.getHID().getYButton());

  private final Trigger DRIVER_POV_UP = new Trigger(m_driverController.povUp());
  private final Trigger DRIVER_POV_DOWN = new Trigger(m_driverController.povDown());
  private final Trigger DRIVER_POV_LEFT = new Trigger(m_driverController.povLeft());
  private final Trigger DRIVER_POV_RIGHT = new Trigger(m_driverController.povRight());

  private final Trigger DRIVER_LEFT_TRIGGER = new Trigger(m_driverController.leftTrigger());
  private final Trigger DRIVER_RIGHT_TRIGGER = new Trigger(m_driverController.rightTrigger());
  private final Trigger DRIVER_LEFT_BUMPER = new Trigger(m_driverController.leftBumper());
  private final Trigger DRIVER_RIGHT_BUMPER = new Trigger(m_driverController.rightBumper());


  // Subsystem definitions
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));
  EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Commands Definitions

 
  //Elevator Commands
  ElevatorManualDown m_elevatorManualDown = new ElevatorManualDown(m_elevatorSubsystem);
  ElevatorManualUp m_elevatorManualUp = new ElevatorManualUp(m_elevatorSubsystem);

  //End Effector Commands
  EndEffectorManualIntake m_endEffectorManualIntake = new EndEffectorManualIntake(m_endEffectorSubsystem);
  EndEffectorManualOuttake m_endEffectorManualOuttake = new EndEffectorManualOuttake(m_endEffectorSubsystem);
  EndEffectorManualPivotDown m_endEffectorManualPivotDown = new EndEffectorManualPivotDown(m_endEffectorSubsystem);
  EndEffectorManualPivotUp m_endEffectorManualPivotUp = new EndEffectorManualPivotUp(m_endEffectorSubsystem);


  Elevator_Setpoint_CMD m_elevatorL1 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL1ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL2 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL2ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL3 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL3ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL4 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL4ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorStow = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kStowElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorCS = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kCSElevatorSetpoint);

  EndEffector_Setpoint_CMD m_endEffectorL1 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL1EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL2 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL2EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL3 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL3EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL4 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL4EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorStow = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorCS = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kCSEndEffectorSetpoint);

  // Sequence Commands
  CS_CMD m_CS_CMD = new CS_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  STOW_CMD m_STOW_CMD = new STOW_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L1_CMD m_L1_CMD = new L1_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L2_CMD m_L2_CMD = new L2_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L3_CMD m_L3_CMD = new L3_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRightX() * 1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(OperatorConstants.ROBOT_SPEED)
                                                            .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  SwerveInputStream driveRobotOriented = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                            getYAxisPOV(),
                                                            getXAxisPOV())
                                                            .withControllerRotationAxis(getRotAxis())
                                                            .scaleTranslation(0.15)
                                                            .allianceRelativeControl(false)
                                                            .robotRelative(true); 
                                                      

  enum RobotState
  {
    L1,L2,L3,
    CORAL_STATION,
    STOW,
    MANUAL_END_EFFECTOR_UP,
    MANUAL_END_EFFECTOR_DOWN,
    MANUAL_ELEVATOR_UP,
    MANUAL_ELEVATOR_DOWN,
    INTAKING_CORAL,
    EJECTING_CORAL
  }

  @AutoLogOutput(key = "RobotState")
  private RobotState m_robotState = RobotState.STOW;
  
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_swerveSubsystem.replaceSwerveModuleFeedforward(0.22234, 2.0995, 0.17259);
  }

  private void updateRobotState(RobotState newState) {
    m_robotState = newState;
    SmartDashboard.putString("Robot_State", m_robotState.toString());
  }

  private DoubleSupplier getXAxisPOV(){
    return () -> {
      if(DRIVER_POV_LEFT.getAsBoolean()) return -1;
      if(DRIVER_POV_RIGHT.getAsBoolean()) return 1;
      return 0;
    };
  }

  private DoubleSupplier getYAxisPOV(){
    return () -> {
      if(DRIVER_POV_DOWN.getAsBoolean()) return 0;
      if(DRIVER_POV_UP.getAsBoolean()) return 0;
      return 0;
    };
  }

  private DoubleSupplier getRotAxis(){
    return () -> {
      if(DRIVER_POV_DOWN.getAsBoolean()) return 0;
      if(DRIVER_POV_UP.getAsBoolean()) return 0;
      return 0;
    };
  }

  // Method to configure bindings
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity); 
    Command driveRobotOrientedNudge  = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);


    if (RobotBase.isSimulation())
    {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      // Simulation driver bindings
      
    }
    if (DriverStation.isTest())
    {
      // Test driver bindings
      m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    } else
    {
      DRIVER_A_BUTTON.onTrue(
        Commands.runOnce(m_swerveSubsystem :: zeroGyro)
      );
      
      // Driver End Effector Manual Intake
    DRIVER_LEFT_TRIGGER.whileTrue(
      Commands.run(() -> {
        m_endEffectorManualIntake.schedule();
        updateRobotState(RobotState.INTAKING_CORAL);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_endEffectorManualIntake.cancel();
        updateRobotState(RobotState.STOW);
      })
    );

    // Driver End Effector Manual Outtake
    DRIVER_RIGHT_TRIGGER.whileTrue(
      Commands.run(() -> {
        m_endEffectorManualOuttake.schedule();
        updateRobotState(RobotState.EJECTING_CORAL);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_endEffectorManualOuttake.cancel();
        updateRobotState(RobotState.STOW);
      })
    );

    // Driver End Effector Manual Pivot Up
    DRIVER_RIGHT_BUMPER.whileTrue(
      Commands.run(() -> {
        m_endEffectorManualPivotUp.schedule();
        updateRobotState(RobotState.MANUAL_END_EFFECTOR_UP);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_endEffectorManualPivotUp.cancel();
        updateRobotState(RobotState.STOW);
      })
    );

    // Driver End Effector Manual Pivot Down
    DRIVER_LEFT_BUMPER.whileTrue(
      Commands.run(() -> {
        m_endEffectorManualPivotDown.schedule();
        updateRobotState(RobotState.MANUAL_END_EFFECTOR_DOWN);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_endEffectorManualPivotDown.cancel();
        updateRobotState(RobotState.STOW);
      })
    );


    // Driver Elevator Stow
    DRIVER_B_BUTTON.whileTrue(
      Commands.run(() -> {
        m_endEffectorStow.schedule();
        updateRobotState(RobotState.STOW);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_endEffectorStow.cancel();
        updateRobotState(RobotState.STOW);
      })
    );

    DRIVER_POV_RIGHT.whileTrue(
      Commands.runOnce(() -> {
        driveRobotOrientedNudge.schedule();
    })
    ).whileFalse(
      Commands.runOnce(() -> {
        driveRobotOrientedNudge.cancel();
      })
    );

    DRIVER_POV_LEFT.whileTrue(
      Commands.runOnce(() -> {
        driveRobotOrientedNudge.schedule();
    })
    ).whileFalse(
      Commands.runOnce(() -> {
        driveRobotOrientedNudge.cancel();
      })
    );

    // Operator L1
    m_operatorController1.button(2).whileTrue(
      Commands.run(() -> {
        m_L1_CMD.schedule();
        m_endEffectorStow.cancel();
        updateRobotState(RobotState.L1);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_L1_CMD.cancel();
        m_endEffectorStow.schedule();
        updateRobotState(RobotState.STOW);
      })
    );

    // m_operatorController1.button(2).whileFalse(
    //     m_endEffectorStow
    //   );


    // Operator L2
    m_operatorController1.button(1).whileTrue(
      Commands.run(() -> {
        m_L2_CMD.schedule();
        m_endEffectorStow.cancel();
        updateRobotState(RobotState.L2);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_L2_CMD.cancel();
        m_endEffectorStow.schedule();
        updateRobotState(RobotState.STOW);
      })
    );

    // m_operatorController1.button(2).whileFalse(
    //     m_endEffectorStow
    // );


    // Operator L3
    m_operatorController1.button(3).whileTrue(
      Commands.run(() -> {
        m_L3_CMD.schedule();
        m_endEffectorStow.cancel();
        updateRobotState(RobotState.L3);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_L3_CMD.cancel();
        m_endEffectorStow.schedule();
        updateRobotState(RobotState.STOW);
      })
    );

    // m_operatorController1.button(3).whileFalse(
    //   m_endEffectorStow
    // );

    // Operator Coral Station
    m_operatorController2.button(11).whileTrue(
      Commands.run(() -> {
        m_CS_CMD.schedule();
        m_endEffectorStow.cancel();
        updateRobotState(RobotState.CORAL_STATION);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_CS_CMD.cancel();
        m_endEffectorStow.schedule();
        updateRobotState(RobotState.STOW);
      })
    );

    // m_operatorController2.button(11).whileFalse(
    //   m_endEffectorStow
    // );

    // Operator Coral Station
    m_operatorController2.button(10).whileTrue(
      Commands.run(() -> {
        m_CS_CMD.schedule();
        m_endEffectorStow.cancel();
        updateRobotState(RobotState.CORAL_STATION);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_CS_CMD.cancel();
        m_endEffectorStow.schedule();
        updateRobotState(RobotState.STOW);
      })
    );

    // m_operatorController2.button(10).whileFalse(
    //     m_endEffectorStow
    //   );


    // Operator Elevator Manual Up
    m_operatorController2.button(12).whileTrue(
      Commands.run(() -> {
        m_elevatorManualUp.schedule();
        updateRobotState(RobotState.MANUAL_ELEVATOR_UP);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_elevatorManualUp.cancel();
        updateRobotState(RobotState.STOW);
      })
    );

    // Operator Elevator Manual Down
    m_operatorController1.button(12).whileTrue(
      Commands.run(() -> {
        m_elevatorManualDown.schedule();
        updateRobotState(RobotState.MANUAL_ELEVATOR_DOWN);
      })
    ).whileFalse(
      Commands.runOnce(() -> {
        m_elevatorManualDown.cancel();
        updateRobotState(RobotState.STOW);
      })
    );
  }
 
      
      
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to runOnce in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be runOnce in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
