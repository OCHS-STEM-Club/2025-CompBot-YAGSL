// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manual.GroundIntake.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundManualRollersOuttake extends Command {
  /** Creates a new GroundOuttake. */
  private CoralGroundIntakeSubsystem m_groundIntakeSubsystem;
  public GroundManualRollersOuttake(CoralGroundIntakeSubsystem groundIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_groundIntakeSubsystem = groundIntakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_groundIntakeSubsystem.groundRollersOuttake();
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
