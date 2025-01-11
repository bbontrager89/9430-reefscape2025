package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AlignCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final Set<Integer> desiredTagIds;

  // You might want some config constants for speeds and distances:
  private static final double ROTATION_SPEED = 0.2; // how fast to rotate
  private static final double STRAFE_SPEED = 0.2;   // how fast to strafe
  private static final double FORWARD_SPEED = 0.5;  // how fast to drive forward/back

  // For final approach distance (e.g. 0.2 meter from the tag)
  private static final double DESIRED_TAG_DISTANCE = 0.2; // in meters

  // Tolerances
  private static final double ANGLE_TOLERANCE_DEG = 1.0;
  private static final double LATERAL_TOLERANCE   = 0.05; // meters
  private static final double DISTANCE_TOLERANCE  = 0.05; // meters

  // Simple state machine
  private enum AlignState {
    DETECT_CAMERA,
    ROTATE,
    ALIGN,
    DRIVE,
    DONE
  }
  private AlignState currentState = AlignState.DETECT_CAMERA;

  // We'll store the camera index that first sees the tag
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
        // Make sure we have a valid tag detection
        int lastTagId = poseEstimatorSubsystem.getLastDetectedTagId();
        if (!desiredTagIds.contains(lastTagId)) {
          // No valid tag in view => do nothing or rotate slowly
          driveSubsystem.drive(0, 0, 0, false);
          return;
        }

        // Record which camera sees it
        detectedCameraIndex = poseEstimatorSubsystem.getLastDetectionCameraIndex();
        System.out.println("[DETECT_CAMERA] Tag " + lastTagId + " seen by cameraIndex=" + detectedCameraIndex);

        // If front camera => skip to ALIGN
        if (detectedCameraIndex == 1) {
          // front camera
          currentState = AlignState.ALIGN;
        } else if (detectedCameraIndex == 0 || detectedCameraIndex == 2) {
          // rotate to bring it into front camera's FOV
          currentState = AlignState.ROTATE;
        }
        break;

      case ROTATE:
        // We want to rotate in place until the front camera picks it up
        // If left camera => rotate left, if right camera => rotate right
        double rotSpeed = (detectedCameraIndex == 0) ? +ROTATION_SPEED : -ROTATION_SPEED;

        driveSubsystem.drive(0, 0, rotSpeed, false);

        // If the front camera picks it up, move to ALIGN
        int currCam = poseEstimatorSubsystem.getLastDetectionCameraIndex();
        int currId  = poseEstimatorSubsystem.getLastDetectedTagId();
        if (currCam == 1 && desiredTagIds.contains(currId)) {
          driveSubsystem.drive(0, 0, 0, false);
          currentState = AlignState.ALIGN;
        }
        break;

      case ALIGN:
        // Now that the front camera sees the tag, we want angle ~ 0, lateral offset ~ 0
        double angleToTag = poseEstimatorSubsystem.getAngleToTagDegrees();
        double lateral = poseEstimatorSubsystem.getLateralOffsetToTag();

        // We'll do a simple P-control approach
        double rotationCmd = 0.0;
        if (Math.abs(angleToTag) > ANGLE_TOLERANCE_DEG) {
          // NOTE: If the robot turns the wrong direction, you might need -angleToTag
          rotationCmd = 0.02 * angleToTag; 
          // clamp the command
          rotationCmd = Math.max(-0.35, Math.min(+0.35, rotationCmd));
        }

        double strafeCmd = 0.0;
        if (Math.abs(lateral) > LATERAL_TOLERANCE) {
          // NOTE: If strafe direction is reversed, you might need -lateral
          strafeCmd = 0.8 * lateral;
          // clamp
          strafeCmd = Math.max(-STRAFE_SPEED, Math.min(STRAFE_SPEED, strafeCmd));
        }

        // Debug prints
        System.out.println("[ALIGN] angleToTag=" + angleToTag
                           + " lateral=" + lateral
                           + " => rotationCmd=" + rotationCmd
                           + " strafeCmd=" + strafeCmd);

        // Drive in robot-relative coords (fwd=0, side=strafeCmd, rot=rotationCmd)
        driveSubsystem.driveRobotRelative(
            new edu.wpi.first.math.kinematics.ChassisSpeeds(
                0.0,
                strafeCmd,
                rotationCmd
            )
        );

        // Check if near aligned
        if (Math.abs(angleToTag) <= ANGLE_TOLERANCE_DEG
            && Math.abs(lateral) <= LATERAL_TOLERANCE) {
          driveSubsystem.drive(0, 0, 0, false);
          currentState = AlignState.DRIVE;
        }
        break;

      case DRIVE:
        // final step: drive to desired distance
        double currentDistance = poseEstimatorSubsystem.getDistanceToTag();
        if (Double.isNaN(currentDistance)) {
          // lost tag? just stop
          driveSubsystem.drive(0, 0, 0, false);
          return;
        }
        double distanceError = currentDistance - DESIRED_TAG_DISTANCE;

        double forwardCmd = 0.0;
        if (Math.abs(distanceError) > DISTANCE_TOLERANCE) {
          // Simple P control
          forwardCmd = 0.8 * distanceError;
          // clamp
          forwardCmd = Math.max(-FORWARD_SPEED, Math.min(FORWARD_SPEED, forwardCmd));
          driveSubsystem.driveRobotRelative(
              new edu.wpi.first.math.kinematics.ChassisSpeeds(
                  forwardCmd,
                  0.0,
                  0.0
              )
          );
        } else {
          // we are in range
          driveSubsystem.drive(0, 0, 0, false);
          currentState = AlignState.DONE;
        }
        break;

      case DONE:
      default:
        // do nothing
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
    // Stop the robot
    driveSubsystem.drive(0, 0, 0, false);
    System.out.println("[AlignCommand] Ended. interrupted=" + interrupted);
  }
}
