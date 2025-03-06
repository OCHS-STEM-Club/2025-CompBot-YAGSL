// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.Manual.EndEffector.Intake.EndEffectorManualIntake;
import frc.robot.commands.Setpoints_CMD.Elevator_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HANDOFF_CMD extends SequentialCommandGroup {
  /** Creates a new L1_CMD. */
  EndEffectorSubsystem m_endEffectorSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  
  public HANDOFF_CMD(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;

    addCommands(
    new ParallelCommandGroup(
                 m_endEffectorSubsystem.intakeWithTOF(),
                 new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kHandoffElevatorSetpoint).until(m_elevatorSubsystem.isAtSetpoint()), 
                 new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kHandoffEndEffectorSetpoint).withTimeout(3),
                 new SequentialCommandGroup(new WaitCommand(1),
                                            new Elevator_Setpoint_CMD(m_elevatorSubsystem, 0.8).until(m_elevatorSubsystem.isAtSetpoint()),
                                            new Elevator_Setpoint_CMD(m_elevatorSubsystem, 5).until(m_elevatorSubsystem.isAtSetpoint()),
                                            new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint))
                                            ));

  }
}
