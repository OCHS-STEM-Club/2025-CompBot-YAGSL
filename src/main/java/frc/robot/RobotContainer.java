// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem = new CoralGroundIntakeSubsystem();
  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Commands Definitions

  //Coral Ground Intake Commands
  CoralGroundManualIntake m_coralGroundManualIntake = new CoralGroundManualIntake(m_coralGroundIntakeSubsystem);
  CoralGroundManualOuttake m_coralGroundManualOuttake = new CoralGroundManualOuttake(m_coralGroundIntakeSubsystem);
  CoralGroundManualPivotDown m_coralGroundManualPivotDown = new CoralGroundManualPivotDown(m_coralGroundIntakeSubsystem);
  CoralGroundManualPivotUp m_coralGroundManualPivotUp = new CoralGroundManualPivotUp(m_coralGroundIntakeSubsystem);

  //Elevator Commands
  ElevatorManualDown m_elevatorManualDown = new ElevatorManualDown(m_elevatorSubsystem);
  ElevatorManualUp m_elevatorManualUp = new ElevatorManualUp(m_elevatorSubsystem);

  //End Effector Commands
  EndEffectorManualIntake m_endEffectorManualIntake = new EndEffectorManualIntake(m_endEffectorSubsystem);
  EndEffectorManualOuttake m_endEffectorManualOuttake = new EndEffectorManualOuttake(m_endEffectorSubsystem);
  EndEffectorManualPivotDown m_endEffectorManualPivotDown = new EndEffectorManualPivotDown(m_endEffectorSubsystem);
  EndEffectorManualPivotUp m_endEffectorManualPivotUp = new EndEffectorManualPivotUp(m_endEffectorSubsystem);


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

      DRIVER_POV_UP.whileTrue(
        m_elevatorManualUp
      );

      DRIVER_POV_DOWN.whileTrue(
        m_elevatorManualDown
      );

      DRIVER_POV_RIGHT.whileTrue(
        m_endEffectorManualPivotUp
      );

      DRIVER_POV_LEFT.whileTrue(
        m_endEffectorManualPivotDown
      );

      DRIVER_LEFT_BUMPER.whileTrue(
        m_endEffectorManualIntake
      );

      DRIVER_RIGHT_BUMPER.whileTrue(
        m_endEffectorManualOuttake
      );

      DRIVER_Y_BUTTON.whileTrue(
        m_coralGroundManualPivotUp
      );

      DRIVER_X_BUTTON.whileTrue(
        m_coralGroundManualPivotDown
      );

      DRIVER_LEFT_TRIGGER.whileTrue(
        m_coralGroundManualIntake
      );

      DRIVER_RIGHT_TRIGGER.whileTrue(
        m_coralGroundManualOuttake
      );


      // m_driverController.y().whileTrue(m_endEffectorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // m_driverController.a().whileTrue(m_endEffectorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // m_driverController.b().whileTrue(m_endEffectorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // m_driverController.x().whileTrue(m_endEffectorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      

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
