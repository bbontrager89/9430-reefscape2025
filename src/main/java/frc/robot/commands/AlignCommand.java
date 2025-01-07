// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.util.Set;

public class AlignCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final PoseEstimatorSubsystem poseEstimator;
  private final Timer timer = new Timer();

  // We’ll define a small state machine
  private enum AlignState {
    WAITING_FOR_TAG,    // No tag (or invalid tag) seen
    ROTATE_FOR_CAM2,    // If camera 2 is not the one seeing it
    ALIGN_ANGLE,        // Rotate to match angle
    DRIVE_FORWARD,      // Drive forward until N distance
    DONE
  }

  private AlignState currentState = AlignState.WAITING_FOR_TAG;

  // How close in angle do we need to be to consider ourselves “aligned”
  private static final double ANGLE_THRESHOLD_DEG = 5.0;
  // Desired distance from the tag (in meters). For example, 1 meter
  private static final double TARGET_DISTANCE_M = 1.0;

  private final Set<Integer> allowedTagIds;

  public AlignCommand(DriveSubsystem driveSubsystem, Set<Integer> allowedTagIds) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimator = driveSubsystem.getPoseEstimatorSubsystem();
    this.allowedTagIds = allowedTagIds;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    currentState = AlignState.WAITING_FOR_TAG;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    int lastCamIndex = poseEstimator.getLastDetectionCameraIndex();
    int lastTagId = poseEstimator.getLastDetectedTagId();
    double lastTimestamp = poseEstimator.getLastDetectionTimestamp();
    double timeSinceDetect = timer.get() - lastTimestamp;

    // If we haven't seen a (valid) tag in some time, revert to WAITING_FOR_TAG
    if (timeSinceDetect > 0.5 || !allowedTagIds.contains(lastTagId)) {
      currentState = AlignState.WAITING_FOR_TAG;
    }

    switch (currentState) {
      case WAITING_FOR_TAG:
        // Check if we see a valid tag
        if (lastCamIndex != -1
            && allowedTagIds.contains(lastTagId)
            && timeSinceDetect <= 0.5) {
          
          // If it’s already camera 2, skip ROTATE_FOR_CAM2
          if (lastCamIndex == 1) {
            currentState = AlignState.ALIGN_ANGLE;
          } else {
            currentState = AlignState.ROTATE_FOR_CAM2;
          }
        } else {
          // Stop the robot
          driveSubsystem.drive(0.0, 0.0, 0.0, false);
        }
        break;

      case ROTATE_FOR_CAM2:
        // Keep rotating in place until lastDetectionCameraIndex == 1
        // We'll rotate at a small speed. For simplicity, let's just rotate +0.2 rad/s
        if (lastCamIndex != 1) {
          driveSubsystem.drive(0.0, 0.0, 0.2, false); // robot-relative
        } else {
          // Now that camera 2 sees it, go to align angle
          currentState = AlignState.ALIGN_ANGLE;
        }
        break;

      case ALIGN_ANGLE:
        double angleDiff = poseEstimator.getAngleToTagDegrees();
        double kP = 0.01; // TUNE THIS!
        double rotationCmd = kP * angleDiff;

        // Clip to some max rotational speed, say +/- 0.3
        if (rotationCmd > 0.3) rotationCmd = 0.3;
        if (rotationCmd < -0.3) rotationCmd = -0.3;

        // If within threshold, go to DRIVE_FORWARD
        if (Math.abs(angleDiff) < ANGLE_THRESHOLD_DEG) {
          currentState = AlignState.DRIVE_FORWARD;
        } else {
          // rotate in place
          driveSubsystem.drive(0.0, 0.0, rotationCmd, false);
        }
        break;

      case DRIVE_FORWARD:
        double dist = poseEstimator.getDistanceToTag();
        if (dist > TARGET_DISTANCE_M + 0.05) {
          // drive forward at some speed, e.g. 0.2 m/s
          driveSubsystem.drive(0.2, 0.0, 0.0, false);
        } else {
          // Close enough
          currentState = AlignState.DONE;
        }
        break;

      case DONE:
      default:
        driveSubsystem.drive(0.0, 0.0, 0.0, false);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop
    driveSubsystem.drive(0.0, 0.0, 0.0, false);
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    // End once we've reached the DONE state
    return (currentState == AlignState.DONE);
  }
}
