// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Functions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Manual.GroundIntake.Rollers.GroundManualRollersIntake;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pulse_cmd extends SequentialCommandGroup {
  /** Creates a new pulse_cmd. */
  CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem;
  public pulse_cmd(CoralGroundIntakeSubsystem coralGroundIntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_coralGroundIntakeSubsystem = coralGroundIntakeSubsystem;
    addCommands(
      new GroundManualRollersIntake(m_coralGroundIntakeSubsystem).withTimeout(0.5),
      Commands.run(()-> m_coralGroundIntakeSubsystem.groundRollersStop())
    );
  }
}
