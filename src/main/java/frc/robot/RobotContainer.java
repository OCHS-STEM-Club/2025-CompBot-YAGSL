// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.CoralGroundIntake.Intake.Manual.CoralGroundManualIntake;
import frc.robot.commands.CoralGroundIntake.Intake.Manual.CoralGroundManualOuttake;
import frc.robot.commands.CoralGroundIntake.Pivot.Manual.CoralGroundManualPivotDown;
import frc.robot.commands.CoralGroundIntake.Pivot.Manual.CoralGroundManualPivotUp;
import frc.robot.commands.Elevator.Manual.ElevatorManualDown;
import frc.robot.commands.Elevator.Manual.ElevatorManualUp;
import frc.robot.commands.EndEffector.Intake.Manual.EndEffectorManualIntake;
import frc.robot.commands.EndEffector.Intake.Manual.EndEffectorManualOuttake;
import frc.robot.commands.EndEffector.Pivot.Manual.EndEffectorManualPivotDown;
import frc.robot.commands.EndEffector.Pivot.Manual.EndEffectorManualPivotUp;
import frc.robot.commands.Sequential.CS_CMD;
import frc.robot.commands.Sequential.L1_CMD;
import frc.robot.commands.Sequential.L2_CMD;
import frc.robot.commands.Sequential.L3_CMD;
import frc.robot.commands.Sequential.L4_CMD;
import frc.robot.commands.Sequential.STOW_CMD;
import frc.robot.commands.Setpoints.ElevatorSetpoints.Elevator_L1;
import frc.robot.commands.Setpoints.ElevatorSetpoints.Elevator_L2;
import frc.robot.commands.Setpoints.ElevatorSetpoints.Elevator_L3;
import frc.robot.commands.Setpoints.ElevatorSetpoints.Elevator_L4;
import frc.robot.commands.Setpoints.ElevatorSetpoints.Elevator_Stow;
import frc.robot.commands.Setpoints.EndEffectorSetpoints.EndEffector_L1;
import frc.robot.commands.Setpoints.EndEffectorSetpoints.EndEffector_L2;
import frc.robot.commands.Setpoints.EndEffectorSetpoints.EndEffector_L3;
import frc.robot.commands.Setpoints.EndEffectorSetpoints.EndEffector_L4;
import frc.robot.commands.Setpoints.EndEffectorSetpoints.EndEffector_Stow;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;
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



  // private final Trigger OPERATOR_A_BUTTON = new Trigger(() -> m_operatorController.getHID().getAButton());
  // private final Trigger OPERATOR_B_BUTTON = new Trigger(() -> m_operatorController.getHID().getBButton());
  // private final Trigger OPERATOR_X_BUTTON = new Trigger(() -> m_operatorController.getHID().getXButton());
  // private final Trigger OPERATOR_Y_BUTTON = new Trigger(() -> m_operatorController.getHID().getYButton());

  // private final Trigger OPERATOR_POV_UP = new Trigger(m_operatorController.povUp());
  // private final Trigger OPERATOR_POV_DOWN = new Trigger(m_operatorController.povDown());
  // private final Trigger OPERATOR_POV_LEFT = new Trigger(m_operatorController.povLeft());
  // private final Trigger OPERATOR_POV_RIGHT = new Trigger(m_operatorController.povRight());

  // private final Trigger OPERATOR_LEFT_TRIGGER = new Trigger(m_operatorController.leftTrigger());
  // private final Trigger OPERATOR_RIGHT_TRIGGER = new Trigger(m_operatorController.rightTrigger());
  // private final Trigger OPERATOR_LEFT_BUMPER = new Trigger(m_operatorController.leftBumper());
  // private final Trigger OPERATOR_RIGHT_BUMPER = new Trigger(m_operatorController.rightBumper());

  // Subsystem definitions
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));
  EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
  // CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem = new CoralGroundIntakeSubsystem();
  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Commands Definitions

  //Coral Ground Intake Commands
  // CoralGroundManualIntake m_coralGroundManualIntake = new CoralGroundManualIntake(m_coralGroundIntakeSubsystem);
  // CoralGroundManualOuttake m_coralGroundManualOuttake = new CoralGroundManualOuttake(m_coralGroundIntakeSubsystem);
  // CoralGroundManualPivotDown m_coralGroundManualPivotDown = new CoralGroundManualPivotDown(m_coralGroundIntakeSubsystem);
  // CoralGroundManualPivotUp m_coralGroundManualPivotUp = new CoralGroundManualPivotUp(m_coralGroundIntakeSubsystem);

  //Elevator Commands
  ElevatorManualDown m_elevatorManualDown = new ElevatorManualDown(m_elevatorSubsystem);
  ElevatorManualUp m_elevatorManualUp = new ElevatorManualUp(m_elevatorSubsystem);

  //End Effector Commands
  EndEffectorManualIntake m_endEffectorManualIntake = new EndEffectorManualIntake(m_endEffectorSubsystem);
  EndEffectorManualOuttake m_endEffectorManualOuttake = new EndEffectorManualOuttake(m_endEffectorSubsystem);
  EndEffectorManualPivotDown m_endEffectorManualPivotDown = new EndEffectorManualPivotDown(m_endEffectorSubsystem);
  EndEffectorManualPivotUp m_endEffectorManualPivotUp = new EndEffectorManualPivotUp(m_endEffectorSubsystem);

  Elevator_L1 m_elevatorL1 = new Elevator_L1(m_elevatorSubsystem);
  Elevator_L2 m_elevatorL2 = new Elevator_L2(m_elevatorSubsystem);
  Elevator_L3 m_elevatorL3 = new Elevator_L3(m_elevatorSubsystem);
  Elevator_L4 m_elevatorL4 = new Elevator_L4(m_elevatorSubsystem);
  Elevator_Stow m_elevatorStow = new Elevator_Stow(m_elevatorSubsystem);

  EndEffector_L1 m_endEffectorL1 = new EndEffector_L1(m_endEffectorSubsystem);
  EndEffector_L2 m_endEffectorL2 = new EndEffector_L2(m_endEffectorSubsystem);
  EndEffector_L3 m_endEffectorL3 = new EndEffector_L3(m_endEffectorSubsystem);
  EndEffector_L4 m_endEffectorL4 = new EndEffector_L4(m_endEffectorSubsystem);
  EndEffector_Stow m_endEffectorStow = new EndEffector_Stow(m_endEffectorSubsystem);

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
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRightX() * 1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.4)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  
 
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

  // Method to configure bindings
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);  

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
        m_endEffectorManualIntake
      );
      // Driver End Effector Manual Outtake
      DRIVER_RIGHT_TRIGGER.whileTrue(
        m_endEffectorManualOuttake
      );
      
      // Driver End Effector Manual Pivot Up
      DRIVER_RIGHT_BUMPER.whileTrue(
        m_endEffectorManualPivotUp
      );
      // Driver End Effector Manual Pivot Down
      DRIVER_LEFT_BUMPER.whileTrue(
        m_endEffectorManualPivotDown
      );
      // Driver Elevator Stow
      DRIVER_B_BUTTON.whileTrue(
        m_endEffectorStow
      );

      // Operator L1
      m_operatorController1.button(2).whileTrue(
        m_L1_CMD
      );
      m_operatorController1.button(2).whileFalse(
        m_endEffectorStow
      );
      // Operator L2
      m_operatorController1.button(1).whileTrue(
        m_L2_CMD
      );
      m_operatorController1.button(1).whileFalse(
        m_endEffectorStow
      );
      
      // Operator L3
      m_operatorController1.button(3).whileTrue(
        m_L3_CMD
      );
      m_operatorController1.button(3).whileFalse(
        m_endEffectorStow
      );
      // Operator Coral Station
      m_operatorController2.button(11).whileTrue(
        m_CS_CMD
      );
      m_operatorController2.button(11).whileFalse(
        m_endEffectorStow
      );
      // Operator Coral Station
      m_operatorController2.button(10).whileTrue(
        m_CS_CMD
      );
      m_operatorController2.button(10).whileFalse(
        m_endEffectorStow
      );



      // Operator Elevator Manual Up
      m_operatorController2.button(12).whileTrue(
        m_elevatorManualUp
      );
      // Operator Elevator Manual Down
      m_operatorController1.button(12).whileTrue(
        m_elevatorManualDown
      );
      


      
      
      
  }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
