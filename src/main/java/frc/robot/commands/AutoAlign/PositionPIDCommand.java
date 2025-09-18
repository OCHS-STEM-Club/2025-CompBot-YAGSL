// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionPIDCommand extends Command {
  /** Creates a new PositionPIDCommand. */
  public SwerveSubsystem m_swerveSubsystem;

  public final Pose2d m_targetPose;
  private PPHolonomicDriveController m_driverController = Constants.AutoAlignConstants.kAutoAlignController;

  private final Timer timer = new Timer();


  private PositionPIDCommand(SwerveSubsystem swerveSubsystem, Pose2d targetPose) {
    m_swerveSubsystem = swerveSubsystem;
    m_targetPose = targetPose;

  }

  public static Command generateCommand(SwerveSubsystem swerveSubsystem, Pose2d targetPose, Time timeout) {
    return new PositionPIDCommand(swerveSubsystem, targetPose)
        .withTimeout(timeout)
        .finallyDo(() -> {
          swerveSubsystem.drive(new ChassisSpeeds(0,0,0));
          swerveSubsystem.lock();
         });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
    targetState.pose = m_targetPose;

    m_swerveSubsystem.drive(
      m_driverController.calculateRobotRelativeSpeeds(m_swerveSubsystem.getPose(), targetState));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}