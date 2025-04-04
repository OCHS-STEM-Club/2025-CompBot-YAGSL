// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualIntake;
import frc.robot.commands.Setpoints_CMD.Elevator_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.GroundIntake_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.Handoff_Elevator_CMD;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HANDOFF_CMD extends SequentialCommandGroup {
  /** Creates a new L1_CMD. */
  EndEffectorSubsystem m_endEffectorSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem;
  
  public HANDOFF_CMD(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, CoralGroundIntakeSubsystem coralGroundIntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_coralGroundIntakeSubsystem = coralGroundIntakeSubsystem;

    addCommands(
    new ParallelCommandGroup(

                 m_endEffectorSubsystem.intakeWithTOF(),
                 new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kBufferElevatorSetpoint).until(m_elevatorSubsystem.isAtSetpoint()), 
                 new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kHandoffEndEffectorSetpoint).withTimeout(3),
                 new GroundIntake_Setpoint_CMD(m_coralGroundIntakeSubsystem, SetpointConstants.kBufferCoralGroundIntakeSetpoint).withTimeout(4),
                 new SequentialCommandGroup(new WaitCommand(1),
                                            new Handoff_Elevator_CMD(m_elevatorSubsystem).until(m_elevatorSubsystem.isAtSetpoint()),
                                            new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kBufferElevatorSetpoint).until(m_elevatorSubsystem.isAtSetpoint()),
                                            new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint)),
                 new SequentialCommandGroup(new WaitCommand(3.7),
                                            new GroundIntake_Setpoint_CMD(coralGroundIntakeSubsystem, SetpointConstants.kStowCoralGroundIntakeSetpoint))
                                            ));

  }

  public Command rerunHandoff(){
    if(m_endEffectorSubsystem.hasCoral() == false && m_coralGroundIntakeSubsystem.getHopperSensor() == true){
      return new HANDOFF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, m_coralGroundIntakeSubsystem);
    }else
      return Commands.none();
  } 
}
