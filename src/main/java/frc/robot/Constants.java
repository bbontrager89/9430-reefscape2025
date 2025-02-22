// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int pigeon2CanId = 1;

    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class VisionConstants {
    public static final int[] kAlignApriltagIDs = new int[] { 6 };
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0729;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // TODO bbontrager89 20241107.1742: Need to update values for
    // kDrivingMotorReduction
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;

    public static final double kTriggerThreshold = 0.1;

    public static final double scoringDistance = 0.02;
    public static final double coralIntakeDistance = 0.12;

    public static final double leftScoringOffset = -0.165;
    public static final double rightScoringOffset = 0.165;

    public static final double algaeOffset = 0.25;

    // TODO tune these offsets to align with the grooves on the intake station
    public static final double intakePositionLeft = -0.3;
    public static final double intakePositionRight = 0.3;

    public static final double doublePressBuffer = 0.5;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ElevatorConstants {

    public static final int elevatorMotorCanId = 27;

    public static final double kP = -20;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double positionTolerence = 0.01;

    public static final double minimumElevatorHeight = 0.175; // Furthest possible is 0.166
    public static final double maximumElevatorHeight = 0.675; // Furthest possible is 0.683

    public static final double coralStationPosition = 0.398; // Scoring position 0
    public static final double level1ScoringPosition = 0.316;
    public static final double level2ScoringPosition = 0.51;
    public static final double level3ScoringPosition = 0.669;

    public static final double algaeClearLevelTwo = 0.46;
    public static final double algaeClearLevelThree = 0.6;

    public static final boolean elevatorMotorInverted = false;

    public static final double encoderToRevolutionRatio = 200.0;

    public static final double rangeInRevolutions = (maximumElevatorHeight - minimumElevatorHeight)
        * encoderToRevolutionRatio;

    public static final int maximumAutoSpeed = 1; // Range: 0 to 1


  }

  public static final class CoralManipulatorConstants {
    public static final int PivotMotorCanId = 42;
    public static final int IntakeMotorCanId = 41;

    public static final double pivotKp = 2.25;
    public static final double pivotKi = 0.5;
    public static final double pivotKd = 0.11;

    public static final double maxPivotSpeed = 0.15;

    public static final double autoStopCurrent = 30;

    public static final double maximumPivotPosition = 0.48;
    public static final double minimumPivotPosition = 0.095;

    public static final double intakePivotPosition = 0.375;
    public static final double levelOnePivotPosition = 0.25;
    public static final double levelTwoPivotPosition = 0.091; // Motor Off: low hard limit
    public static final double levelThreePivotPosition = 0.091;

    public static final double algaeClear = 0.17;
  }

  public static final class AprilTagConstants {
    public static final int[] scoringAprilTags = new int[] {
        6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22
    };

    public static final int[] intakeStationAprilTags = new int[] {
        1, 2, 12, 13
    };
  }

}
