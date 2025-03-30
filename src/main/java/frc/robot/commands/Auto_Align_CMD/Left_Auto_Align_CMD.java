// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto_Align_CMD;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Left_Auto_Align_CMD extends Command {
  /** Creates a new Left_Auto_Align_CMD. */

  private PhotonCamera m_ReefCamera = new PhotonCamera("Reef_CAM");

  private SwerveSubsystem m_swerveSubsystem;

  private DoubleSupplier m_Translation_X;
  private DoubleSupplier m_Translation_Y;
  private DoubleSupplier m_Rotation;

  private double m_Target_YAW = 0;//TODO: Set this to the target yaw find setpoint
  private double m_Current_YAW;
  private double m_Error_YAW;

  private ProfiledPIDController m_ProfiledPIDController = 
                                new ProfiledPIDController(
                                 0,
                                 0,
                                 0,
                                 new TrapezoidProfile.Constraints(1, 0.05));   
                                 
  private CommandXboxController m_driverController = new CommandXboxController(0);

  public Left_Auto_Align_CMD(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      var results = m_ReefCamera.getAllUnreadResults();
      if (!results.isEmpty()) {
        
        var latestResult = results.get(results.size() - 1);
        if(latestResult.hasTargets()){
          var bestTarget = latestResult.getBestTarget();

          m_Current_YAW = bestTarget.getYaw();

        }
      }


      m_Error_YAW = m_Current_YAW - m_Target_YAW;

      m_Translation_X = ()-> m_ProfiledPIDController.calculate(-m_Error_YAW);
      m_Translation_Y= ()-> 0.4;
      m_Rotation = ()-> m_driverController.getRightX() * -1;

      m_swerveSubsystem.driveRobotOriented(m_Translation_Y, m_Translation_X, m_Rotation);
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
