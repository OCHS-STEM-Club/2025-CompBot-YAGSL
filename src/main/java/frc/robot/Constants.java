// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

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
    public static double kCurrentRobotTranslationSpeed = 0.4;
    public static double kCurrentRobotRotationSpeed = 1;

    public static final double kNormalRobotTranslationSpeed = 0.4;
    public static final double kNormalRobotRotationSpeed = 1;

    public static final double kReducedRobotTranslationSpeed = 0.1;
    public static final double kReducedRobotRotationSpeed = 0.25;
    
    public static final double kRobotNudgeSpeed = 0.15;
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
    // End Effector IDs
    public static final int kEndEffectorIntakeID = 23;
    public static final int kEndEffectorPivotID = 22;
    public static final int kEndEffectorTOFID = 27;
    // End Effector Speeds
    public static final double kEndEffectorSpeed = 0.50;
    public static final double kPivotSpeed = 0.15;
    // End Effector TOF Detection Value
    public static final double kEndEffectorTOFDetectionValue = 81;
    // End Effector PID Values
    public static final double kEndEffectorPivotPIDValueP = 38.029;
    public static final double kEndEffectorPivotPIDValueI = 0;
    public static final double kEndEffectorPivotPIDValueD = 0;
    public static final double kEndEffectorFeedForward = 0.280975;
    // End Effector Beam Break ID
    public static final int kEndEffectorBeamBreakPort = 3;
    // CANdi ID
    public static final int kCANdiID = 26;
    // CANdi Cofigs
    public static final double kPWM1AbsoluteEncoderOffset = -0.13; 
    public static final double kPWM1AbsoluteEncoderDiscontinuityPoint = 1; 
    // Sensor Configs
    public static final double kSensorToMechanismRatio = 1;
    public static final double kRotorToSensorRatio = 24.9;
    // Pivot Current Limits
    public static final double kEndEffectorPivotCurrentLimit = 40;
    //End Effector Motion Magic Values
    public static final double kEndEffectorPivotMotionMagicCruiseVelocity = 2185;
    public static final double kEndEffectorPivotMotionMagicAcceleration = 1000;
    public static final double kEndEffectorPivotMotionMagicJerk = 1700;
    // Soft Limit Values
    public static final double kEndEffectorFowardSoftLimit = 0.8;
    public static final double kEndEffectorReverseSoftLimit = 0.15;


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
    public static final double kElevatorMotionMagicAcceleration = 40633;
    public static final double kElevatorMotionMagicCruiseVelocity = 35633;
    public static final double kElevatorMotionMagicJerk = 45000;
    // Elevator Speed
    public static final double kElevatorSpeed = 0.15;
    public static final double kElevatorVoltage = 1.2;
    // Elevator Limits
    public static final int kBottomElevatorLimitPort = 5;


  }

  public static class GroundIntakeConstants {

    public static final int kGroundIntakeMotorID = 21;
    public static final int kGroundIntakePivotID = 20;

    public static final int kGroundIntakeBeamBreakPort = 8;
    public static final int kHopperBeamBreakPort = 9;
    
    public static final double kGroundRollersSpeed = 0.5;
    public static final double kGroundPivotSpeed = 0.3;

    public static final double kGroundPivotPIDValueP = 30;
    public static final double kGroundPivotPIDValueI = 0;
    public static final double kGroundPivotPIDValueD = 0;

    public static final double kGroundIntakePivotFeedForward = 0.022094;
    
    public static final double kGroundIntakePivotMotionMagicCruiseVelocity = 5461;
    public static final double kGroundIntakePivotMotionMagicAcceleration = 6425;
    public static final double kGroundIntakePivotMotionMagicJerk = 0;

    public static final double kGroundIntakeEncoderOffset = 0;
    public static final double kGroundIntakeDiscontinuityPoint = 1;

    public static final double kSensorToMechanismRatio = 1;
    public static final double kRotorToSensorRatio = 80;
  }

  public static class SetpointConstants{

    public static final double kL1ElevatorSetpoint = 0; 
    public static final double kL1EndEffectorSetpoint = 0.296875; 

    public static final double kL2ElevatorSetpoint = 7.166; 
    public static final double kL2EndEffectorSetpoint = 0.17; 

    public static final double kL3ElevatorSetpoint = 12.218; 
    public static final double kL3EndEffectorSetpoint = 0.175;//17

    public static final double kL4ElevatorSetpoint = 20.370; 
    public static final double kL4EndEffectorSetpoint = 0.16;//0.1445;

    public static final double kHPElevatorSetpoint = 3.05; 
    public static final double kHPEndEffectorSetpoint = 0.643; 

    public static final double kStowElevatorSetpoint = 0; 
    public static final double kStowEndEffectorSetpoint = 0.55; 
    public static final double kStowCoralGroundIntakeSetpoint = 0.65; 
    
    public static final double kHandoffElevatorSetpoint = 0.8; 
    public static final double kHandoffEndEffectorSetpoint = 1.01; 

    public static final double kBufferElevatorSetpoint = 5; 
    public static final double kBufferCoralGroundIntakeSetpoint = 0.5; 



    public static final double kSetpointThreshold = 0.75;


    
  }

  public static class VisionConstants {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final String FL_Module_Camera_Name = "FL-Module";
        public static final Transform3d FL_Module_Camera_Transformed =
                        new Transform3d(new Translation3d(0.21082, -0.23876, 0.2032), 
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(343)));
        public static final Matrix<N3, N1> FL_SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> FL_MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);


        public static final String FR_Module_Camera_Name = "FR-Module";

        public static final Transform3d FR_Module_Camera_Transformed =
                        new Transform3d(new Translation3d(0.21082, 0.23876, 0.2032), 
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(17)));
        public static final Matrix<N3, N1> FR_SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> FR_MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  }

  public static class LEDConstants {
    public static final int kLEDCount = 69;
    public static final int kCANdiID = 25;
  }
  

}
