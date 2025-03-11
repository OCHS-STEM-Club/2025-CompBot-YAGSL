// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.Sequential.Intaking_CMDs.GI_Intake_Sequence;
import frc.robot.commands.Sequential.Intaking_CMDs.HP_Intake_Sequence;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private CoralGroundIntakeSubsystem m_coralGroundIntakeSubsystem;
  private EndEffectorSubsystem m_endEffectorSubsystem;
  private ElevatorSubsystem m_elevatorSubsystem;
  private SwerveSubsystem m_swerveSubsystem;
  private RobotContainer m_robotContainer;

  private GI_Intake_Sequence m_GI_Intake_Sequence;
  private HP_Intake_Sequence m_HP_Intake_Sequence;

  private CANdle m_CANdle;

  public enum LED_States {
    EE_Has_Coral,
    GI_Has_Coral,
    Algae_Removal,
    Handoff_Coral,
    Stow
    
  }


  public LEDSubsystem(CoralGroundIntakeSubsystem coralGroundIntakeSubsystem,
                      EndEffectorSubsystem endEffectorSubsystem,
                      ElevatorSubsystem elevatorSubsystem,
                      SwerveSubsystem swerveSubsystem, 
                      RobotContainer robotContainer,
                      GI_Intake_Sequence GI_Intake_Sequence,
                      HP_Intake_Sequence HP_Intake_Sequence) {

    m_coralGroundIntakeSubsystem = coralGroundIntakeSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    m_robotContainer = robotContainer;

    m_GI_Intake_Sequence = GI_Intake_Sequence;
    m_HP_Intake_Sequence = HP_Intake_Sequence;

    m_CANdle = new CANdle(LEDConstants.kCANdiID);
  }

  public void setCANdle(LED_States toChange){

    switch (toChange) {
      case EE_Has_Coral:
        m_CANdle.setLEDs(255, 255, 255); // White
        break;
      case GI_Has_Coral:
        m_CANdle.animate(new StrobeAnimation(247, 181, 0, 255, 0.1, LEDConstants.kLEDCount)); // Traffic Yellow
        break;
      case Algae_Removal:
        m_CANdle.setLEDs(0, 220, 100); // Algae Color
        break;
      case Handoff_Coral:
        m_CANdle.animate(new StrobeAnimation(54, 1, 63, 255, 0.1, LEDConstants.kLEDCount)); // Purple
        break;
      case Stow:
        m_CANdle.animate(new ColorFlowAnimation(0, 57, 162, 255, 0.65, LEDConstants.kLEDCount, Direction.Forward)); // Philippine Blue
        break;
    }
  }

  @Override
  public void periodic() {
    if(m_endEffectorSubsystem.hasCoral()){
      setCANdle(LED_States.EE_Has_Coral);
    }else if(m_coralGroundIntakeSubsystem.getIntakeSensor()){
      setCANdle(LED_States.GI_Has_Coral);
    }else if(m_robotContainer.m_L2_Algae_Removal.isScheduled() || m_robotContainer.m_L3_Algae_Removal.isScheduled()){
      setCANdle(LED_States.Algae_Removal);
    }else if (m_GI_Intake_Sequence.m_handoffCMD.isScheduled() || m_HP_Intake_Sequence.m_handoffCMD.isScheduled()){
      setCANdle(LED_States.Handoff_Coral);
    }else{
      setCANdle(LED_States.Stow);
    }
  }
}


