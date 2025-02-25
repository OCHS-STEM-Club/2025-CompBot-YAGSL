// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    // Robot Speed
    public static final double ROBOT_SPEED     = 0.4;


  }

  public static class EndEffectorConstants 
  {
    // End Effector IDs
    public static final int kEndEffectorIntakeID = 23;
    public static final int kEndEffectorPivotID = 22;
    // End Effector Speeds
    public static final double kEndEffectorSpeed = 0.95;
    public static final double kPivotSpeed = 0.15;
    // End Effector Current Spike
    public static final double kEndEffectorCurrentSpike = 110;
    // End Effector PID Values
    public static final double kEndEffectorPivotPIDValueP = 38.029;
    public static final double kEndEffectorPivotPIDValueI = 0;
    public static final double kEndEffectorPivotPIDValueD = 0;
    // public static final double kEndEffectorPivotPIDValueS = 0;
    // public static final double kEndEffectorPivotPIDValueV = 3.7822;
    // public static final double kEndEffectorPivotPIDValueA = 1.3466;
    // public static final double kEndEffectorPivotPIDValueG = 2.4047;
    // End Effector Beam Break ID
    public static final int kEndEffectorBeamBreakPort = 3;
    // CANdi ID
    public static final int kCANdiID = 26;
    // CANdi Cofigs
    public static final double kPWM1AbsoluteEncoderOffset = -0.74511;//-0.74511 // 0.245
    public static final double kPWM1AbsoluteEncoderDiscontinuityPoint = 0.98; // 0.98
    // Pivot Current Limits
    public static final double kEndEffectorPivotCurrentLimit = 40;

    public static final double kEndEffectorPivotMotionMagicCruiseVelocity = 2185;
    public static final double kEndEffectorPivotMotionMagicCruiseAcceleration = 1000;
    public static final double kEndEffectorPivotMotionMagicCruiseJerk = 1700;

  }

  public static class CoralGroundIntakeConstants 
  {
    // Coral Ground Intake IDs
    public static final int kCoralGroundIntakeID = 21;
    public static final int kCoralGroundPivotID = 20;
    // Coral Ground Intake Speeds
    public static final double kCoralGroundIntakeSpeed = 0.25;
    public static final double kCoralGroundPivotSpeed = 0.25;
    // Coral Ground Intake PID Values
    public static final double kCoralGroundPivotPIDValueP = 0;
    public static final double kCoralGroundPivotPIDValueI = 0;
    public static final double kCoralGroundPivotPIDValueD = 0;
    // Coral Ground Beam Break ID
    public static final int kCoralGroundBeamBreakPort = 2;

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
    public static final double kElevatorMotionMagicAcceleration = 30633;
    public static final double kElevatorMotionMagicCruiseVelocity = 35633;
    public static final double kElevatorMotionMagicJerk = 45000;

    // Elevator Speed
    public static final double kElevatorSpeed = 0.15;
    // Elevator Limits
    public static final int kTopElevatorLimitPort = 5;
    public static final int kBottomElevatorLimitPort = 7;


  }

  public static class SetpointConstants{
    public static final double kL1ElevatorSetpoint = 0; //0
    public static final double kL1EndEffectorSetpoint = 0.4; //0.4

    public static final double kL2ElevatorSetpoint = 9.25; //10.5
    public static final double kL2EndEffectorSetpoint = 0.19; //0.19

    public static final double kL3ElevatorSetpoint = 13.55; //14.8
    public static final double kL3EndEffectorSetpoint = 0.2; //0.2

    public static final double kL4ElevatorSetpoint = 0; //0
    public static final double kL4EndEffectorSetpoint = 0; //0

    public static final double kCSElevatorSetpoint = 3.05; // Used to be 4 rotations. Increased to 5.6 for better intaking from coral station. Dropped to 4.3 rotations. last values was too high.
    public static final double kCSEndEffectorSetpoint = 0.643; //0.643

    public static final double kStowElevatorSetpoint = 0; //0
    public static final double kStowEndEffectorSetpoint = 0.5; //0.5

    public static final double kSetpointThreshold = 0.75;

    
  }

  public static class VisionConstants {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);


        public static final String FL_Module_Camera_Name = "FL-Module";
        public static final Transform3d FL_Module_Camera_Transformed =
                        new Transform3d(new Translation3d(23622, -0.24638, 0), 
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(15)));
        public static final Matrix<N3, N1> FL_SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> FL_MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);


        public static final String FR_Module_Camera_Name = "FR-Module";

        public static final Transform3d FR_Module_Camera_Transformed =
                        new Transform3d(new Translation3d(0.23622, 0.24638, 0.5), 
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(345)));
        public static final Matrix<N3, N1> FR_SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> FR_MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);



  }
}
