// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

public class DriveSubsystem extends SubsystemBase {
  // Each of these is one of our swerve modules (the wheels + motors that can spin
  // and turn).
  // We have four modules: front-left, front-right, back-left, and back-right.
  public final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  public final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  public final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  public final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // This is our gyro sensor. It tells us which way the robot is facing.
  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.pigeon2CanId);

  // The PoseEstimatorSubsystem is like a fancy brain that combines odometry
  // (where the wheels think we are)
  // with vision (where the cameras think we are) and the gyro (which way we're
  // facing)
  // to come up with a single "best guess" of our position on the field.
  private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    zeroHeading(); // Reset the gyro so we start facing 'forward' as zero degrees.

    RobotConfig config = new RobotConfig(24.13, 6.883,
        new ModuleConfig(0.0362,
            4.46, 1.2, DCMotor.getNeoVortex(4),
            60.0, 4),
        new Translation2d[] { new Translation2d(0.286, 0.286), new Translation2d(0.286, -0.286),
            new Translation2d(-0.286, 0.286), new Translation2d(-0.286, -0.286) });
    try {
      // config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // AutoBuilder is using our pose and drive methods to run autonomous paths.
    // Now, instead of using odometry directly, it uses the pose from
    // poseEstimatorSubsystem.
    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new com.pathplanner.lib.config.PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID
                                                                                                // constants
            new com.pathplanner.lib.config.PIDConstants(AutoConstants.kPYController, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    // Drive the robot at the given speeds relative to itself (not the field).
    chassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Figure out how fast we're moving from the states of each swerve module.
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
  }

  @Override
  public void periodic() {
    // Every loop, we ask the PoseEstimatorSubsystem to update its idea of where we
    // are.
    // We give it the gyro reading and all swerve wheel positions.
    poseEstimatorSubsystem.update(pigeon.getRotation2d(), getSwervePositions());
  }

  /**
   * Returns the currently-estimated fused pose of the robot (i.e., the position
   * that combines all sensors).
   */
  public Pose2d getPose() {
    return poseEstimatorSubsystem.getEstimatedPose();
  }

  /**
   * Resets the robot's known position on the field.
   * This might be used at the start of a match if we know exactly where we start.
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimatorSubsystem.resetPose(pose, pigeon.getRotation2d(), getSwervePositions());
  }

  /**
   * Drive method for controlling the robot with joystick values.
   * fieldRelative = true means we interpret the stick directions as field
   * directions (north/south),
   * false means the stick directions are relative to the robot's front.
   * rateLimit = true tries to smooth out sudden changes in direction.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    //TODO: FIX TO USE TO ROBOT RELATIVE SPEEDS
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                pigeon.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Lock the robot’s wheels in an "X" shape to help keep it from being pushed
   * around when stopped.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Set the wheels to some desired states directly.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Reset encoders if you need to recalibrate your wheel positions. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Reset gyro heading to zero, often done at the start of the match. */
  public void zeroHeading() {
    pigeon.reset();
  }

  /** Get current heading in degrees, -180 to 180. */
  public double getHeading() {
    return pigeon.getRotation2d().getDegrees();
  }

  /** Get how fast we are turning (degrees per second). */
  public double getTurnRate() {
    return pigeon.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Get the swerve modules' current positions (how far each wheel has rolled).
   */
  private SwerveModulePosition[] getSwervePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

}
