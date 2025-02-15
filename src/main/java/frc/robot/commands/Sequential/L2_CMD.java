// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Setpoints.ElevatorSetpoints.Elevator_L2;
import frc.robot.commands.Setpoints.EndEffectorSetpoints.EndEffector_Stow;
import frc.robot.commands.Setpoints.EndEffectorSetpoints.EndEffector_L2;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L2_CMD extends SequentialCommandGroup {
  /** Creates a new L2_CMD. */
  EndEffectorSubsystem m_endEffectorSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  
  public L2_CMD(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;

    addCommands( new EndEffector_Stow(m_endEffectorSubsystem).withTimeout(2),
                 new Elevator_L2(m_elevatorSubsystem).withTimeout(2), 
                 new EndEffector_L2(m_endEffectorSubsystem));
  }
}
