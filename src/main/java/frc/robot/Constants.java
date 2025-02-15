// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
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
    public static final double DEADBAND        = 0.2;
    public static final double LEFT_Y_DEADBAND = 0.2;
    public static final double RIGHT_X_DEADBAND = 0.2;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class EndEffectorConstants 
  {
    // End Effector IDs
    public static final int kEndEffectorIntakeID = 23;
    public static final int kEndEffectorPivotID = 22;
    // End Effector Speeds
    public static final double kEndEffectorSpeed = 0.75;
    public static final double kPivotSpeed = 0.15;
    // End Effector Current Spike
    public static final double kEndEffectorCurrentSpike = 110;
    // End Effector PID Values
    public static final double kEndEffectorPivotPIDValueP = 38.029;
    public static final double kEndEffectorPivotPIDValueI = 0;
    public static final double kEndEffectorPivotPIDValueD = 0;
    public static final double kEndEffectorPivotPIDValueS = 0;
    public static final double kEndEffectorPivotPIDValueV = 3.7822;
    public static final double kEndEffectorPivotPIDValueA = 1.3466;
    public static final double kEndEffectorPivotPIDValueG = 2.4047;
    // End Effector Beam Break ID
    public static final int kEndEffectorBeamBreakPort = 3;
    // CANdi ID
    public static final int kCANdiID = 26;
    // CANdi Cofigs
    public static final double kPWM1AbsoluteEncoderOffset = -0.74511;
    public static final double kPWM1AbsoluteEncoderDiscontinuityPoint = 1;
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
    public static final double kElevatorPIDValueP = 0;
    public static final double kElevatorPIDValueI = 0;
    public static final double kElevatorPIDValueD = 0;
    public static final double kElevatorPIDValueS = 0;
    public static final double kElevatorPIDValueV = 0;
    public static final double kElevatorPIDValueA = 0;
    public static final double kElevatorPIDValueG = 0;
    // Sensor To Mechanism Ratio
    public static final double kElevatorSensorToMechRatio = 5;
    // Motion Magic Configs
    public static final double kElevatorMotionMagicAcceleration = 0;
    public static final double kElevatorMotionMagicCruiseVelocity = 0;
    // Elevator Speed
    public static final double kElevatorSpeed = 0.15;
    // Elevator Limits
    public static final int kTopElevatorLimitPort = 6;
    public static final int kBottomElevatorLimitPort = 7;
    // ElevatorSetpointThreshold
    public static final double kElevatorSetpointThreshold = 0.5;

  }
}
