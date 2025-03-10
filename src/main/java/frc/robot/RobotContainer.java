// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.fasterxml.jackson.core.util.ReadConstrainedTextBuffer;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.Functions.Coral_Intake_CMD;
import frc.robot.commands.Manual.Elevator.ElevatorManualDown;
import frc.robot.commands.Manual.Elevator.ElevatorManualUp;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualIntake;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualOuttake;
import frc.robot.commands.Manual.EndEffector.Pivot.EndEffectorManualPivotDown;
import frc.robot.commands.Manual.EndEffector.Pivot.EndEffectorManualPivotUp;
import frc.robot.commands.Manual.GroundIntake.Pivot.GroundIntakeManualPivotDown;
import frc.robot.commands.Manual.GroundIntake.Pivot.GroundIntakeManualPivotUp;
import frc.robot.commands.Manual.GroundIntake.Rollers.GroundManualRollersIntake;
import frc.robot.commands.Manual.GroundIntake.Rollers.GroundManualRollersOuttake;
import frc.robot.commands.Sequential.HANDOFF_CMD;
import frc.robot.commands.Sequential.STOW_CMD;
import frc.robot.commands.Sequential.Intaking_CMDs.GI_Intake_Sequence;
import frc.robot.commands.Sequential.Intaking_CMDs.HP_Intake_Sequence;
import frc.robot.commands.Sequential.Intaking_CMDs.L1_Intake_Sequence;
import frc.robot.commands.Sequential.Reef.L1_CMD;
import frc.robot.commands.Sequential.Reef.L2_CMD;
import frc.robot.commands.Sequential.Reef.L3_CMD;
import frc.robot.commands.Sequential.Reef.L4_CMD;
import frc.robot.commands.Sequential.Reef.REEF_CMD;
import frc.robot.commands.Setpoints_CMD.Elevator_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.GroundIntake_Setpoint_CMD;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
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

  private enum IntakeMethod{
    HP_INTAKE,
    GI_INTAKE,
    L1_INTAKE
  }
  private IntakeMethod intakeMethodValue = IntakeMethod.GI_INTAKE;

  // Controller definitions
  public CommandXboxController m_driverController = new CommandXboxController(0);
  CommandJoystick m_operatorController1 = new CommandJoystick(1);
  CommandJoystick m_operatorController2 = new CommandJoystick(2);

  // CommandXboxController m_operatorButtonBox = new CommandXboxController(2);


  private final Trigger DRIVER_A_BUTTON = new Trigger(() -> m_driverController.getHID().getAButton());
  private final Trigger DRIVER_B_BUTTON = new Trigger(() -> m_driverController.getHID().getBButton());
  private final Trigger DRIVER_X_BUTTON = new Trigger(() -> m_driverController.getHID().getXButton());
  private final Trigger DRIVER_Y_BUTTON = new Trigger(() -> m_driverController.getHID().getYButton());

  private final Trigger DRIVER_POV_UP = new Trigger(m_driverController.povUp());
  private final Trigger DRIVER_POV_DOWN = new Trigger(m_driverController.povDown());
  private final Trigger DRIVER_POV_LEFT = new Trigger(m_driverController.povLeft());
  private final Trigger DRIVER_POV_RIGHT = new Trigger(m_driverController.povRight());

  public final Trigger DRIVER_LEFT_TRIGGER = new Trigger(m_driverController.leftTrigger());
  private final Trigger DRIVER_RIGHT_TRIGGER = new Trigger(m_driverController.rightTrigger());
  private final Trigger DRIVER_LEFT_BUMPER = new Trigger(m_driverController.leftBumper());
  private final Trigger DRIVER_RIGHT_BUMPER = new Trigger(m_driverController.rightBumper());


  // Subsystem definitions
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));
  EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
  ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem = new CoralGroundIntakeSubsystem();





  //Elevator Manual Commands
  ElevatorManualDown m_elevatorManualDown = new ElevatorManualDown(m_elevatorSubsystem);
  ElevatorManualUp m_elevatorManualUp = new ElevatorManualUp(m_elevatorSubsystem);

  //End Effector Manual Commands
  EndEffectorManualIntake m_endEffectorManualIntake = new EndEffectorManualIntake(m_endEffectorSubsystem);
  EndEffectorManualOuttake m_endEffectorManualOuttake = new EndEffectorManualOuttake(m_endEffectorSubsystem);
  EndEffectorManualPivotDown m_endEffectorManualPivotDown = new EndEffectorManualPivotDown(m_endEffectorSubsystem);
  EndEffectorManualPivotUp m_endEffectorManualPivotUp = new EndEffectorManualPivotUp(m_endEffectorSubsystem);

  // Ground Intake Manual Commands
  GroundIntakeManualPivotUp m_groundManualIntakePivotUp = new GroundIntakeManualPivotUp(m_coralGroundIntakeSubsystem);
  GroundIntakeManualPivotDown m_groundManualIntakePivotDown = new GroundIntakeManualPivotDown(m_coralGroundIntakeSubsystem);
  GroundManualRollersIntake m_groundManualRollersIntake = new GroundManualRollersIntake(m_coralGroundIntakeSubsystem);
  GroundManualRollersOuttake m_groundManualRollersOuttake = new GroundManualRollersOuttake(m_coralGroundIntakeSubsystem);

  // Elevator Setpoint Commands
  Elevator_Setpoint_CMD m_elevatorL1 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL1ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL2 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL2ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL3 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL3ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorL4 = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kL4ElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorStow = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kStowElevatorSetpoint);
  Elevator_Setpoint_CMD m_elevatorHP = new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kHPElevatorSetpoint);

  // End Effector Setpoint Commands
  EndEffector_Setpoint_CMD m_endEffectorL1 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL1EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL2 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL2EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL3 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL3EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorL4 = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kL4EndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorStow = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint);
  EndEffector_Setpoint_CMD m_endEffectorHP = new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kHPEndEffectorSetpoint);
  // Sequence Commands
  public HP_Intake_Sequence m_HP_Intake_Sequence = new HP_Intake_Sequence(m_elevatorSubsystem, m_endEffectorSubsystem,m_coralGroundIntakeSubsystem);
  public GI_Intake_Sequence m_GI_Intake_Sequence = new GI_Intake_Sequence(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem);
  L1_Intake_Sequence m_L1_Intake_Sequence = new L1_Intake_Sequence(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem); 

  STOW_CMD m_STOW_CMD = new STOW_CMD(m_elevatorSubsystem, m_endEffectorSubsystem);
  L1_CMD m_L1_CMD = new L1_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem);
  L2_CMD m_L2_CMD = new L2_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem);
  L3_CMD m_L3_CMD = new L3_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem);
  L4_CMD m_L4_CMD = new L4_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem);

  HANDOFF_CMD m_HANDOFF_CMD = new HANDOFF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem);

  Coral_Intake_CMD m_Coral_Intake_CMD = new Coral_Intake_CMD(m_coralGroundIntakeSubsystem);
  // Algae Removal Commands
  public REEF_CMD m_L3_Algae_Removal = new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem,m_coralGroundIntakeSubsystem, 0.28, 8);
  public REEF_CMD m_L2_Algae_Removal = new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem,m_coralGroundIntakeSubsystem, 0.25, 5);
  // GI Setpoint Commands
  GroundIntake_Setpoint_CMD m_GI_Intake_Setpoint = new GroundIntake_Setpoint_CMD(m_coralGroundIntakeSubsystem, 0.37);
  GroundIntake_Setpoint_CMD m_GI_Stow_Setpoint = new GroundIntake_Setpoint_CMD(m_coralGroundIntakeSubsystem, 0.65);
  GroundIntake_Setpoint_CMD m_GI_STOW_CMD = new GroundIntake_Setpoint_CMD(m_coralGroundIntakeSubsystem, 0.60);

  LEDSubsystem m_ledSubsystem = new LEDSubsystem(m_coralGroundIntakeSubsystem, m_endEffectorSubsystem, m_elevatorSubsystem, m_swerveSubsystem,
                                                this,this.m_GI_Intake_Sequence , this.m_HP_Intake_Sequence);



  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRightX() * 1)
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(SpeedConstants.kCurrentRobotTranslationSpeed)
                                                            .scaleRotation(SpeedConstants.kCurrentRobotRotationSpeed)
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
                                                            .scaleTranslation(SpeedConstants.kRobotNudgeSpeed)
                                                            .allianceRelativeControl(false)
                                                            .robotRelative(true); 
                                                          
                                                      


 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // HPintake = ()-> true;
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_swerveSubsystem.replaceSwerveModuleFeedforward(OperatorConstants.kSSwerveFeedforward, OperatorConstants.kVSwerveFeedforward, OperatorConstants.kASwerveFeedforward);
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

  private Command getDesiredIntakeCMD(){
    return 
    switch (intakeMethodValue) {
      case HP_INTAKE -> m_HP_Intake_Sequence;
      case GI_INTAKE -> m_GI_Intake_Sequence;
      case L1_INTAKE -> m_L1_Intake_Sequence;
    };
  }
  @AutoLogOutput(key = "General/IntakeMethod")
  private String getIntakeMethodString(){
    return intakeMethodValue.toString();
  }

  

  // Method to configure bindings
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity); 
    Command driveRobotOrientedNudge = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);


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
    // Driver Controls

      DRIVER_A_BUTTON.onTrue(
        Commands.runOnce(m_swerveSubsystem :: zeroGyro)
      );
      
      // Driver Elevator Stow
      DRIVER_B_BUTTON.whileTrue(
        Commands.run(() -> {
          m_endEffectorStow.schedule();
        })
      ).whileFalse(
        Commands.runOnce(() -> {
          m_endEffectorStow.cancel();
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

      DRIVER_X_BUTTON.whileTrue(
        Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_GI_Intake_Setpoint.schedule();
      })
      ).whileFalse(
        Commands.runOnce(() -> {
          m_GI_Intake_Setpoint.cancel();
        })
      );

      DRIVER_Y_BUTTON.whileTrue(
        Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_GI_Stow_Setpoint.schedule();
      })
      ).whileFalse(
        Commands.runOnce(() -> {
          m_GI_Stow_Setpoint.cancel();
        })
      );

     DRIVER_LEFT_BUMPER.whileTrue(
        Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_HP_Intake_Sequence.schedule();
      })
      ).whileFalse(
        Commands.runOnce(() -> {
          m_HP_Intake_Sequence.cancel();
          m_endEffectorStow.schedule();
        })
      );

      DRIVER_RIGHT_TRIGGER.whileTrue(m_groundManualRollersOuttake);
      DRIVER_POV_UP.whileTrue(m_endEffectorManualIntake);
  
      DRIVER_RIGHT_BUMPER.whileTrue(m_endEffectorManualOuttake);

      m_driverController.button(8).onTrue(Commands.runOnce(()->CommandScheduler.getInstance().cancelAll()));


      DRIVER_LEFT_TRIGGER.onTrue(
        Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            getDesiredIntakeCMD().schedule();

      })
      );



    // Operator controls

        // Operator L1
        m_operatorController1.button(OperatorConstants.kButtonBox_L1_Button_Port1).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            new InstantCommand(()->{
              intakeMethodValue = intakeMethodValue.L1_INTAKE;
            }).schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_endEffectorStow.schedule();
          })
        );


      // Operator L2
        m_operatorController1.button(OperatorConstants.kButtonBox_L2_Button_Port1).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_GI_STOW_CMD.schedule();
            m_L2_CMD.schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_L2_CMD.cancel();
            m_elevatorManualDown.schedule();
            m_GI_Stow_Setpoint.schedule();
            m_endEffectorStow.schedule();
          })
        );

        // Operator L3
        m_operatorController1.button(OperatorConstants.kButtonBox_L3_Button_Port1).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_GI_STOW_CMD.schedule();
            m_L3_CMD.schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_L3_CMD.cancel();
            m_GI_Stow_Setpoint.schedule();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );


        // Operator L4
        m_operatorController2.button(OperatorConstants.kButtonBox_L4_Button_Port2).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_GI_STOW_CMD.schedule();
            m_L4_CMD.schedule();
           
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_L4_CMD.cancel();
            m_GI_Stow_Setpoint.schedule();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );

        // Operator DA2
        m_operatorController1.button(OperatorConstants.kButtonBox_L2_Button_Port1).and(m_operatorController2.button(7)).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_endEffectorManualIntake.schedule();
            m_L2_Algae_Removal.schedule();
            
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_GI_Stow_Setpoint.schedule();
            m_elevatorManualDown.schedule();
            m_endEffectorStow.schedule();
          })
        );

        // Operator DA3
        m_operatorController1.button(OperatorConstants.kButtonBox_L3_Button_Port1).and(m_operatorController2.button(7)).whileTrue(
          Commands.run(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_endEffectorManualIntake.schedule();
            
            m_L3_Algae_Removal.schedule();
            
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            m_elevatorManualDown.schedule();
            m_GI_Stow_Setpoint.schedule();
            m_endEffectorStow.schedule();
          })
        );


        // m_operatorController2.button(OperatorConstants.kButtonBox_HP_Button_Port2).onTrue(
        //   new InstantCommand(()->{
        //     intakeMethodValue = intakeMethodValue.HP_INTAKE;
        //   }));

        m_operatorController2.button(OperatorConstants.kButtonBox_GI_Button_Port2).onTrue(
          new InstantCommand(()->{
            if(intakeMethodValue == intakeMethodValue.GI_INTAKE)
              intakeMethodValue = intakeMethodValue.HP_INTAKE;
            else if(intakeMethodValue == intakeMethodValue.HP_INTAKE){
              intakeMethodValue = intakeMethodValue.GI_INTAKE;
            }else if(intakeMethodValue == intakeMethodValue.L1_INTAKE){
              intakeMethodValue = intakeMethodValue.HP_INTAKE;

            }
            
          }));


        // Operator Elevator Manual Up
        m_operatorController2.button(OperatorConstants.kButtonBox_ELEVATOR_MANUAL_UP_Button_Port2).whileTrue(
          Commands.run(() -> {
            m_elevatorManualUp.schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_elevatorManualUp.cancel();
          })
        );

        // Operator Elevator Manual Down
        m_operatorController1.button(OperatorConstants.kButtonBox_ELEVATOR_MANUAL_DOWN_Button_Port1).whileTrue(
          Commands.run(() -> {
            m_elevatorManualDown.schedule();
          })
        ).whileFalse(
          Commands.runOnce(() -> {
            m_elevatorManualDown.cancel();
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
