package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AlignCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final Set<Integer> desiredTagIds;

  // Increased rotation speeds
  private static final double ROTATION_SPEED = 0.3; // Increased from 0.2 for faster rotation
  private static final double STRAFE_SPEED = 0.5;
  private static final double FORWARD_SPEED = 0.5;
  
  // Increased rotation gain for faster alignment
  private static final double ROTATION_GAIN = 0.04; // Increased from 0.02 for more aggressive rotation

  private static final double DESIRED_TAG_DISTANCE = 0.2; // in meters

  // Tolerances
  private static final double ANGLE_TOLERANCE_DEG = 5.0;
  private static final double LATERAL_TOLERANCE = 0.05; // meters
  private static final double DISTANCE_TOLERANCE = 0.05; // meters

  private enum AlignState {
    DETECT_CAMERA,
    ROTATE,
    ALIGN,
    DRIVE,
    DONE
  }

  private AlignState currentState = AlignState.DETECT_CAMERA;
  private int detectedCameraIndex = -1;

  public AlignCommand(DriveSubsystem driveSubsystem, Set<Integer> desiredTagIds) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimatorSubsystem = driveSubsystem.getPoseEstimatorSubsystem();
    this.desiredTagIds = desiredTagIds;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    currentState = AlignState.DETECT_CAMERA;
    detectedCameraIndex = -1;
    System.out.println("[AlignCommand] Initialized. Looking for tag IDs: " + desiredTagIds);
  }

  @Override
  public void execute() {
    switch (currentState) {
      case DETECT_CAMERA:
        int lastTagId = poseEstimatorSubsystem.getLastDetectedTagId();
        if (!desiredTagIds.contains(lastTagId)) {
          driveSubsystem.drive(0, 0, 0, false);
          return;
        }

        detectedCameraIndex = poseEstimatorSubsystem.getLastDetectionCameraIndex();
        System.out.println("[DETECT_CAMERA] Tag " + lastTagId + " seen by cameraIndex=" + detectedCameraIndex);

        if (detectedCameraIndex == 0) {
          currentState = AlignState.ALIGN;
        } else if (detectedCameraIndex == 0 || detectedCameraIndex == 2) {
          currentState = AlignState.ROTATE;
        }
        break;

      case ROTATE:
        // Using higher rotation speed for faster initial alignment
        double rotSpeed = (detectedCameraIndex == 0) ? +ROTATION_SPEED : -ROTATION_SPEED;
        driveSubsystem.drive(0, 0, rotSpeed, false);

        int currCam = poseEstimatorSubsystem.getLastDetectionCameraIndex();
        int currId = poseEstimatorSubsystem.getLastDetectedTagId();
        if (currCam == 0 && desiredTagIds.contains(currId)) {
          driveSubsystem.drive(0, 0, 0, false);
          currentState = AlignState.ALIGN;
        }
        break;

      case ALIGN:
        int currentCam = poseEstimatorSubsystem.getLastDetectionCameraIndex();
        if (currentCam != 1) {
          System.out.println("[ALIGN] Warning: Lost front camera! Now on camera " + currentCam);
          driveSubsystem.drive(0, 0, 0, false);
          return;
        }

        double angleToTag = poseEstimatorSubsystem.getAngleToTagDegrees();
        double lateral = poseEstimatorSubsystem.getLateralOffsetToTag();

        // Calculate rotation and strafe commands with increased gains
        double rotationCmd = 0.0;
        if (Math.abs(angleToTag) > ANGLE_TOLERANCE_DEG) {
          rotationCmd = -ROTATION_GAIN * angleToTag;  // Using higher gain
          rotationCmd = Math.max(-0.5, Math.min(+0.5, rotationCmd));  // Increased limits from 0.35
        }

        double strafeCmd = 0.0;
        if (Math.abs(lateral) > LATERAL_TOLERANCE) {
          strafeCmd = lateral;
          strafeCmd = Math.max(-STRAFE_SPEED, Math.min(STRAFE_SPEED, strafeCmd));
        }

        System.out.println("[ALIGN] Status: " +
            "\n  angleToTag=" + angleToTag +
            "\n  lateral=" + lateral +
            "\n  Commands:" +
            "\n    rotation=" + rotationCmd +
            "\n    strafe=" + strafeCmd);

        driveSubsystem.driveRobotRelative(
            new edu.wpi.first.math.kinematics.ChassisSpeeds(
                0.0,
                strafeCmd,
                rotationCmd));

        if (Math.abs(angleToTag) <= ANGLE_TOLERANCE_DEG &&
            Math.abs(lateral) <= LATERAL_TOLERANCE) {
          driveSubsystem.drive(0, 0, 0, false);
          currentState = AlignState.DRIVE;
        }
        break;

      case DRIVE:
        double currentDistance = poseEstimatorSubsystem.getDistanceToTag();
        if (Double.isNaN(currentDistance)) {
          driveSubsystem.drive(0, 0, 0, false);
          return;
        }
        double distanceError = currentDistance - DESIRED_TAG_DISTANCE;

        double forwardCmd = 0.0;
        if (Math.abs(distanceError) > DISTANCE_TOLERANCE) {
          forwardCmd = distanceError;
          forwardCmd = Math.max(-FORWARD_SPEED, Math.min(FORWARD_SPEED, forwardCmd));

          System.out.println("[DRIVE] Status: " +
              "\n  distance=" + currentDistance +
              "\n  error=" + distanceError +
              "\n  command=" + forwardCmd);

          driveSubsystem.driveRobotRelative(
              new edu.wpi.first.math.kinematics.ChassisSpeeds(
                  forwardCmd,
                  0.0,
                  0.0));
        } else {
          driveSubsystem.drive(0, 0, 0, false);
          currentState = AlignState.DONE;
        }
        break;

      case DONE:
      default:
        driveSubsystem.drive(0, 0, 0, false);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return (currentState == AlignState.DONE);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
    System.out.println("[AlignCommand] Ended. interrupted=" + interrupted);
  }
}