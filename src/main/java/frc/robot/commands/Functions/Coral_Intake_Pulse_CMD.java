// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Functions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Manual.GroundIntake.Rollers.GroundManualRollersIntake;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Coral_Intake_Pulse_CMD extends Command {
  /** Creates a new GroundIntake. */
  private CoralGroundIntakeSubsystem m_groundIntakeSubsystem;
  
  public Coral_Intake_Pulse_CMD(CoralGroundIntakeSubsystem groundIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_groundIntakeSubsystem = groundIntakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      new GroundManualRollersIntake(m_groundIntakeSubsystem).withTimeout(0.5).andThen(
      Commands.run(()-> m_groundIntakeSubsystem.groundRollersStop()).withTimeout(0.2));

    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundIntakeSubsystem.groundRollersStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}