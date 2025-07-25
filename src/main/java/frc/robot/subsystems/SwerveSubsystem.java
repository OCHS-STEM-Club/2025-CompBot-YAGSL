// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionCamera;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  private final SwerveDrive swerveDrive;
  
  private VisionCamera[] camerasArray = new VisionCamera[2];

  private boolean enableVision = true;

  private SwerveInputStream m_swerveInputStream;

  List<Pose3d> tagPoses = new LinkedList<>();


  public SwerveSubsystem(File directory)
  {
    


    camerasArray[0] = new VisionCamera(VisionConstants.FL_Module_Camera_Name,
                                       VisionConstants.FL_Module_Camera_Transformed,
                                       VisionConstants.FL_SingleTagStdDevs,
                                       VisionConstants.FL_MultiTagStdDevs);
                                       

    camerasArray[1] = new VisionCamera(VisionConstants.HP_Module_Camera_Name,
                                       VisionConstants.HP_Module_Camera_Transformed,
                                       VisionConstants.HP_SingleTagStdDevs,
                                       VisionConstants.HP_MultiTagStdDevs);
    
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(false,
                                               false,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
//    swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    setupPathPlanner();
    


  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }

  /**
   * Setup the photon vision class.
   */

  @Override
  public void periodic()
  { 
    if(enableVision){
      setupPhotonCameras();
      // logTags();
    }
    

  }

  @Override
  public void simulationPeriodic()
  {
  }





  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(3.8, 0.0, 0),
              // Translation PID constants
              new PIDConstants(5, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }



  public void setupPhotonCameras(){

    for(VisionCamera camera : camerasArray){

      Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();

      if(poseEst.isPresent()){

        var pose = poseEst.get();

        var stdDevs = camera.getEstimationStdDevs();

        swerveDrive.addVisionMeasurement(new Pose2d(pose.estimatedPose.getX(),pose.estimatedPose.getY(), pose.estimatedPose.getRotation().toRotation2d()),
                                         pose.timestampSeconds,
                                         stdDevs
                                        );
      }


    }
                
  }

  // public void logTags(){
  //   if(camerasArray[1].camera.getLatestResult().hasTargets()){
  //     int  HP_BestTarget = camerasArray[1].camera.getLatestResult().getBestTarget().getFiducialId();

  //     var HP_tagPose = Constants.VisionConstants.kTagLayout.getTagPose(HP_BestTarget);

  //     if(HP_tagPose.isPresent()){
  //     tagPoses.add(HP_tagPose.get());
  //   }
  //   }



  //   if(camerasArray[0].camera.getLatestResult().hasTargets()){
  //     int  FL_BestTarget = camerasArray[0].camera.getLatestResult().getBestTarget().getFiducialId();

  //     var FL_tagPose = Constants.VisionConstants.kTagLayout.getTagPose(FL_BestTarget);

  //     if(FL_tagPose.isPresent()){
  //     tagPoses.add(FL_tagPose.get());
  //   }
  //   }





  //   Logger.recordOutput("Best_Targets", tagPoses.toArray(new Pose3d[tagPoses.size()]));

  // }

    






  
  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        1, 1,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }


  public Command drive_To_Reef_A() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_A");
    } catch (Exception e) {
      System.out.println("Path not found_A");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_B() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_B");
    } catch (Exception e) {
      System.out.println("Path not found_B");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    
     PathPlannerPath path1 = new PathPlannerPath(
            path.getWaypoints(), 
            constraints,
            new IdealStartingState(getVelocityMagnitude(swerveDrive.getFieldVelocity()), swerveDrive.getOdometryHeading()), 
            new GoalEndState(0.0, path.getGoalEndState().rotation())
        );
        
    return AutoBuilder.followPath(path);
  }
  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }



  public Command drive_To_Reef_C() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_C");
    } catch (Exception e) {
      System.out.println("Path not found_C");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }


  public Command drive_To_Reef_D() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_D");
    } catch (Exception e) {
      System.out.println("Path not found_D");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_E() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_E");
    } catch (Exception e) {
      System.out.println("Path not found_E");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_F() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_F");
    } catch (Exception e) {
      System.out.println("Path not found_F");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_G() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_G");
    } catch (Exception e) {
      System.out.println("Path not found_G");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_H() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_H");
    } catch (Exception e) {
      System.out.println("Path not found_H");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_I() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_I");
    } catch (Exception e) {
      System.out.println("Path not found_I");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_J() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_J");
    } catch (Exception e) {
      System.out.println("Path not found_J");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_K() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_K");
    } catch (Exception e) {
      System.out.println("Path not found_K");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public Command drive_To_Reef_L() {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile("Reef_L");
    } catch (Exception e) {
      System.out.println("Path not found_L");
    }

    if (path == null) {
      return Commands.none();
    }

    PathConstraints constraints = new PathConstraints(
        3, 3,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
        
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }









  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 9, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  @AutoLogOutput(key = "Subsystems/SwerveSubsystem/Pose2d")
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
