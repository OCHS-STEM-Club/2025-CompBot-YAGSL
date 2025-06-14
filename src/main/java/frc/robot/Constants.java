// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(19.5);


  public static class SpeedConstants{
    // Robot Speed

    public static final double kNormalRobotTranslationSpeed = 0.55;
    public static final double kNormalRobotRotationSpeed = 0.65;
    
    public static final double kRobotNudgeSpeed = 0.1;
  }
  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double kDeadband = 0.1;
  
    // Swerve Feedforward Values
    public static final double kSSwerveFeedforward = 0.2612;
    public static final double kVSwerveFeedforward = 2.1245;
    public static final double kASwerveFeedforward = 0.28492;

    // ButtonBox Button IDs
    public static final int kButtonBox_L1_Button_Port1 = 2;
    public static final int kButtonBox_L2_Button_Port1 = 1;
    public static final int kButtonBox_L3_Button_Port1 = 3;
    public static final int kButtonBox_L4_Button_Port2 = 2;
    public static final int kButtonBox_HP_Button_Port2 = 11;
    public static final int kButtonBox_GI_Button_Port2 = 10;
    public static final int kButtonBox_ELEVATOR_MANUAL_UP_Button_Port2 = 12;
    public static final int kButtonBox_ELEVATOR_MANUAL_DOWN_Button_Port1 = 12;



  }

  public static class EndEffectorConstants 
  {
    // Merge Change
    // End Effector IDs
    public static final int kEndEffectorIntakeID = 23;
    public static final int kEndEffectorPivotID = 22;
    public static final int kEndEffectorTOFID = 27;
    public static final int kEndEffectorEncoderID = 29;

    // End Effector Speeds
    public static final double kEndEffectorSpeed = 0.50;
    public static final double kPivotSpeed = 0.15;
    // End Effector TOF Detection Value
    public static final double kEndEffectorTOFDetectionValue = 95;//85
    // End Effector PID Values
    public static final double kEndEffectorPivotPIDValueP = 38.029;
    public static final double kEndEffectorPivotPIDValueI = 0;
    public static final double kEndEffectorPivotPIDValueD = 0;
    public static final double kEndEffectorFeedForward = 0.280975;
    // End Effector Beam Break ID
    public static final int kEndEffectorBeamBreakPort = 3;
    // Sensor Configs
    public static final double kSensorToMechanismRatio = 1;
    public static final double kRotorToSensorRatio = 24.9; // TODO: Change Setpoints based on this value
    public static final double kEncoderOffset = 0.345;

    // Pivot Current Limits
    public static final double kEndEffectorPivotCurrentLimit = 40;
    //End Effector Motion Magic Values
    public static final double kEndEffectorPivotMotionMagicCruiseVelocity = 5;
    public static final double kEndEffectorPivotMotionMagicAcceleration = 2;
    public static final double kEndEffectorPivotMotionMagicJerk = 1600;
    // Soft Limit Values
    public static final double kEndEffectorFowardSoftLimit = 0.8;
    public static final double kEndEffectorReverseSoftLimit = 0.15;

    public static final double kEndEffectorCurrentSpike = 55;
  }

  public static class ElevatorConstants{
    // Elevator Motor IDs
    public static final int kElevatorRightMotorID = 18;
    public static final int kElevatorLeftMotorID = 19;

    // Elevator PID Values
    public static final double kElevatorPIDValueP = 3.596;
    public static final double kElevatorPIDValueI = 0;
    public static final double kElevatorPIDValueD = 0;
    public static final double kElevatorPIDValueS = 0;
    public static final double kElevatorPIDValueV = 0.21303;
    public static final double kElevatorPIDValueA = 0;
    public static final double kElevatorPIDValueG = 1.2061;
    // Sensor To Mechanism Ratio
    public static final double kElevatorSensorToMechRatio = 0.875;
    // Motion Magic Configs
    public static final double kElevatorMotionMagicAcceleration = 45;
    public static final double kElevatorMotionMagicCruiseVelocity = 160;
    public static final double kElevatorMotionMagicJerk = 1600;
    // Elevator Speed
    public static final double kElevatorSpeed = 0.15; //0.125
    public static final double kElevatorVoltage = 1.2;
    // Elevator Limits
    public static final int kBottomElevatorLimitPort = 5;


  }

  public static class GroundIntakeConstants {

    public static final int kGroundIntakeMotorID = 21;
    public static final int kGroundIntakePivotID = 20;

    public static final int kGroundIntakeBeamBreakPort = 8;
    public static final int kHopperBeamBreakPort = 9;
    
    public static final double kGroundRollersSpeedIntakeSpeed = 0.5;
    public static final double kGroundRollersSpeedOuttakeSpeed = 0.45;
    public static final double kGroundPivotSpeed = 0.3;

    public static final double kGroundPivotPIDValueP = 30;
    public static final double kGroundPivotPIDValueI = 0;
    public static final double kGroundPivotPIDValueD = 0;

    public static final double kGroundIntakePivotFeedForward = 0.022094;
    
    public static final double kGroundIntakePivotMotionMagicCruiseVelocity = 100;
    public static final double kGroundIntakePivotMotionMagicAcceleration = 10;
    public static final double kGroundIntakePivotMotionMagicJerk = 0;

    public static final double kGroundIntakeEncoderOffset = 0;
    public static final double kGroundIntakeDiscontinuityPoint = 1;

    public static final double kSensorToMechanismRatio = 1;
    public static final double kRotorToSensorRatio = 80;
  }

  public static class ClimberConstants {
    public static final int kClimberMotorID = 24;
    public static final int kClimberEncoderID = 28;

    public static final double kEncoderOffset = 0;

    public static final double kClimberSpeed = 0.8;

  }

  public static class SetpointConstants{ // Change EE Values to match new gear ratio
    // L1
    public static final double kL1ElevatorSetpoint = 0; 
    public static final double kL1EndEffectorSetpoint = 0.266875; 

    public static final double kL1EndEffectorSetpointAlt = 0.5;
    // L2
    public static final double kL2ElevatorSetpoint = 7.25; 
    public static final double kL2EndEffectorSetpoint = 0.152; 
    // L3
    public static final double kL3ElevatorSetpoint = 12.18; 
    public static final double kL3EndEffectorSetpoint = 0.148;//17
    // L4
    public static final double kL4ElevatorSetpoint = 20.370; 
    public static final double kL4EndEffectorSetpoint = 0.123;//0.1445;
    // HP
    public static final double kHPElevatorSetpoint = 3.05; 
    public static final double kHPEndEffectorSetpoint = 0.613; 
    // STOW
    public static final double kStowElevatorSetpoint = 0; 
    public static final double kStowEndEffectorSetpoint = 0.52; 
    public static final double kStowCoralGroundIntakeSetpoint = 0.65; 
    // Handoff
    public static final double kHandoffElevatorSetpoint = 0.7; 
    public static final double kHandoffEndEffectorSetpoint = 1; 
    // Buffer
    public static final double kBufferElevatorSetpoint = 5; 
    public static final double kBufferCoralGroundIntakeSetpoint = 0.5; 
    // GI Setpoints
    public static final double kCoralIntakeSetpoint = 0.39;
    // DA2
    public static final double kEndEffectorL2AlgaeRemovalSetpoint = 0.3;
    public static final double kElevatorL2AlgaeRemovalSetpoint = 3.5;
    // DA3
    public static final double kEndEffectorL3AlgaeRemovalSetpoint = 0.3;
    public static final double kElevatorL3AlgaeRemovalSetpoint = 8;
    // Climb
    public static final double kElevatorClimbSetpoint = 4;

    // EE HP
    public static final double kEndEffectorHPIntakeSetpoint = 0.646;
    public static final double kElevatorHPIntakeSetpoint = 5;




    public static final double kSetpointThreshold = 0.25;


    
  }

  public static class VisionConstants {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final String FL_Module_Camera_Name = "FL-Module";
        public static final Transform3d FL_Module_Camera_Transformed =
                        new Transform3d(new Translation3d(0.21082, 0.23876, 0.2032), 
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(343)));
        public final static Matrix<N3, N1> FL_SingleTagStdDevs = VecBuilder.fill(6, 6, 8);
        public final static Matrix<N3, N1> FL_MultiTagStdDevs = VecBuilder.fill(3, 2, 1);


        public static final String FR_Module_Camera_Name = "FR-Module";

        public static final Transform3d FR_Module_Camera_Transformed =
                        new Transform3d(new Translation3d(0.21082, -0.23876, 0.2032), 
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(17)));
        public final static Matrix<N3, N1> FR_SingleTagStdDevs = VecBuilder.fill(6, 6, 8);
        public final static Matrix<N3, N1> FR_MultiTagStdDevs = VecBuilder.fill(3, 2, 1);

        public static final String HP_Module_Camera_Name = "HP-Camera";

        public static final Transform3d HP_Module_Camera_Transformed = 
                        new Transform3d(new Translation3d(Units.inchesToMeters(-6.305),Units.inchesToMeters(1.1426), Units.inchesToMeters(37.24)), 
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(180)));
        public final static Matrix<N3, N1> HP_SingleTagStdDevs = VecBuilder.fill(6, 6, 8);
        public final static Matrix<N3, N1> HP_MultiTagStdDevs = VecBuilder.fill(3, 2, 1);

  }

  public static class LEDConstants {
    public static final int kLEDCount = 69;
    public static final int kCANdiID = 25;
  }
  

}
