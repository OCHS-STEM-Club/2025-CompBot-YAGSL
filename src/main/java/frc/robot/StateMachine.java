// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.Sequential.Reef.REEF_CMD;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class StateMachine extends SubsystemBase {
  /** Creates a new RobotState. */

  public enum ReefState {
    STOW, L1, L2, L3, L4
  }

  private ReefState currentReefState;

  private ElevatorSubsystem m_elevatorSubsystem;
  private EndEffectorSubsystem m_endEffectorSubsystem;


  public StateMachine(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
    currentReefState  = ReefState.STOW;
  }

  public void setDesiredReefState(ReefState desiredReefState){
    currentReefState = desiredReefState;
  }

  public ReefState getCurrentReefState(){
    return currentReefState;
  }

  public Command executeReefState(ReefState desiredReefState){
    switch (desiredReefState) {
      case L1:
          return new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, SetpointConstants.kL1EndEffectorSetpoint, SetpointConstants.kL1ElevatorSetpoint);
      case L2:
          return new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, SetpointConstants.kL2EndEffectorSetpoint, SetpointConstants.kL2ElevatorSetpoint);
      case L3:
          return new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, SetpointConstants.kL3EndEffectorSetpoint, SetpointConstants.kL3ElevatorSetpoint);
      case L4:
          return new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, SetpointConstants.kL4EndEffectorSetpoint, SetpointConstants.kL4ElevatorSetpoint);
      case STOW:
      default:
          return new REEF_CMD(m_elevatorSubsystem, m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint, SetpointConstants.kStowElevatorSetpoint);
  }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
