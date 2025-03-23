// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential.Intaking_CMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.Manual.GroundIntake.Rollers.GroundManualRollersIntake;
import frc.robot.commands.Sequential.HANDOFF_CMD;
import frc.robot.commands.Setpoints_CMD.Elevator_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.EndEffector_Setpoint_CMD;
import frc.robot.commands.Setpoints_CMD.GroundIntake_Setpoint_CMD;
import frc.robot.subsystems.CoralGroundIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HP_Intake_Sequence extends SequentialCommandGroup {
  /** Creates a new L1_CMD. */
  EndEffectorSubsystem m_endEffectorSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem;
  public HANDOFF_CMD m_handoffCMD;

  
  public HP_Intake_Sequence(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, CoralGroundIntakeSubsystem coralGroundIntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_coralGroundIntakeSubsystem = coralGroundIntakeSubsystem;
    m_handoffCMD = new HANDOFF_CMD(elevatorSubsystem, endEffectorSubsystem, coralGroundIntakeSubsystem);


 
    addCommands(
      new ParallelCommandGroup(
        new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, SetpointConstants.kStowEndEffectorSetpoint).until(()-> m_handoffCMD.isScheduled()),
          new SequentialCommandGroup(
              new ParallelCommandGroup(
                        new GroundIntake_Setpoint_CMD(m_coralGroundIntakeSubsystem, 0.67).until(m_coralGroundIntakeSubsystem.getHopperSensorSupplier()),
                        new GroundManualRollersIntake(m_coralGroundIntakeSubsystem).until(m_coralGroundIntakeSubsystem.getHopperSensorSupplier())),
              Commands.run(()-> m_handoffCMD.schedule())
          )));


    // addCommands(
    //   new ParallelCommandGroup(
    //     new EndEffector_Setpoint_CMD(m_endEffectorSubsystem, 1.01).until(()-> m_endEffectorSubsystem.hasCoral()),
    //     new Elevator_Setpoint_CMD(m_elevatorSubsystem, 5).until(m_coralGroundIntakeSubsystem.getHopperSensorSupplier()),
    //       new SequentialCommandGroup(
    //           new ParallelCommandGroup(
    //                     new GroundIntake_Setpoint_CMD(m_coralGroundIntakeSubsystem, 0.67).until(m_coralGroundIntakeSubsystem.getHopperSensorSupplier()),
    //                     new GroundManualRollersIntake(m_coralGroundIntakeSubsystem).until(m_coralGroundIntakeSubsystem.getHopperSensorSupplier())),
    //           new ParallelCommandGroup( 
    //             new GroundIntake_Setpoint_CMD(m_coralGroundIntakeSubsystem, SetpointConstants.kBufferCoralGroundIntakeSetpoint).until(()-> m_endEffectorSubsystem.hasCoral()),
    //             new Elevator_Setpoint_CMD(m_elevatorSubsystem, SetpointConstants.kHandoffElevatorSetpoint).until(()-> m_endEffectorSubsystem.hasCoral()),
    //             m_endEffectorSubsystem.intakeWithTOF().until(()-> m_endEffectorSubsystem.hasCoral())
    //           )
    //       )));
    

}
}
