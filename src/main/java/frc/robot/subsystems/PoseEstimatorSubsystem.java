// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.io.UncheckedIOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    // Arrays for multiple cameras
    private PhotonCamera[] photonCameras;
    private PhotonPoseEstimator[] photonPoseEstimators;

    // These constants describe how much uncertainty we think we have in our measurements.
    // Lower numbers mean we trust that measurement more.
    private static final double STATE_STD_DEV_POS = 0.03;
    private static final double STATE_STD_DEV_HEADING = 0.02;
    private static final double VISION_STD_DEV_POS = 0.9;
    private static final double VISION_STD_DEV_HEADING = 0.9;

    private static final String[] CAMERA_NAMES = {"Arducam_1", "Arducam_2", "Arducam_3"};

    // Each camera might be mounted differently on the robot.
    // These transforms describe the camera’s position/orientation relative to robot center.
    private final Transform3d[] robotToCams = {
        new Transform3d(Inches.of(-10.5), Inches.of(0), Inches.of(6.5), new Rotation3d(0, 2.00713, 4.71239)),
        new Transform3d(Inches.of(0), Inches.of(10.5), Inches.of(6.5), new Rotation3d(0, 2.00713, 0)),
        new Transform3d(Inches.of(10.5), Inches.of(0), Inches.of(6.5), new Rotation3d(0, 2.00713, 1.5708))
    };

    private int lastDetectedTagId = -1;
    private int lastDetectionCameraIndex = -1;
    private double lastDetectionTimestamp = -1.0;
    // Distance from robot center to the tag (in meters)
    private double distanceToTag = Double.NaN;
    // Angle difference between robot’s heading (in degrees) and the line from robot center to tag
    private double angleToTagDegrees = Double.NaN;

    public PoseEstimatorSubsystem(Pose2d initialPose) {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        } catch (UncheckedIOException e) {
            e.printStackTrace();
            System.err.println("Failed to load AprilTagFieldLayout. We'll run without vision corrections.");
            aprilTagFieldLayout = null;
        }

        photonCameras = new PhotonCamera[CAMERA_NAMES.length];
        photonPoseEstimators = new PhotonPoseEstimator[CAMERA_NAMES.length];

        if (aprilTagFieldLayout != null) {
            for (int i = 0; i < CAMERA_NAMES.length; i++) {
                photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
                photonPoseEstimators[i] = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCams[i]
                );
            }
        } else {
            for (int i = 0; i < CAMERA_NAMES.length; i++) {
                photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
            }
        }

        var stateStdDevs = VecBuilder.fill(STATE_STD_DEV_POS, STATE_STD_DEV_POS, STATE_STD_DEV_HEADING);
        var visionStdDevs = VecBuilder.fill(VISION_STD_DEV_POS, VISION_STD_DEV_POS, VISION_STD_DEV_HEADING);

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            initialPose,
            stateStdDevs,
            visionStdDevs
        );
    }

    /**
     * Called periodically by the DriveSubsystem to update our pose from wheel/gyro
     * and optionally from vision (AprilTags).
     */
    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        // 1. Update the pose from odometry
        poseEstimator.update(gyroRotation, modulePositions);

        // 2. Attempt to use each camera's detections
        if (photonPoseEstimators != null && aprilTagFieldLayout != null) {
            for (int i = 0; i < photonCameras.length; i++) {
                PhotonCamera camera = photonCameras[i];
                PhotonPoseEstimator estimator = photonPoseEstimators[i];
                if (estimator == null) {
                    continue;
                }

                List<PhotonPipelineResult> results = camera.getAllUnreadResults();
                for (PhotonPipelineResult result : results) {
                    if (result.hasTargets()) {
                        Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
                        if (estimatedPose.isPresent()) {
                            EstimatedRobotPose camPose = estimatedPose.get();
                            poseEstimator.addVisionMeasurement(
                                camPose.estimatedPose.toPose2d(),
                                camPose.timestampSeconds
                            );

                            // We’ll just record the FIRST target in this result
                            var firstTarget = result.getBestTarget();
                            if (firstTarget != null) {
                                this.lastDetectedTagId = firstTarget.getFiducialId();
                                this.lastDetectionCameraIndex = i;
                                this.lastDetectionTimestamp = camPose.timestampSeconds;

                                // Now we can compute distance & angle from the robot’s pose to the tag’s known field pose:
                                Pose2d robotPose = poseEstimator.getEstimatedPosition();

                                Optional<Pose3d> tagPose3d = aprilTagFieldLayout.getTagPose(this.lastDetectedTagId);
                                if (tagPose3d.isPresent()) {
                                    Pose2d tagPose2d = tagPose3d.get().toPose2d();

                                    // Vector from robot to tag in field coords:
                                    Translation2d diff = tagPose2d.getTranslation().minus(robotPose.getTranslation());
                                    // Distance:
                                    this.distanceToTag = diff.getNorm();

                                    // Robot heading:
                                    double robotHeadingDeg = robotPose.getRotation().getDegrees();
                                    // Angle from robot position to tag in field coordinates:
                                    double angleToTag = Math.toDegrees(Math.atan2(
                                        diff.getY(),
                                        diff.getX()
                                    ));

                                    // We want difference in heading:
                                    double angleDiff = angleToTag - robotHeadingDeg;
                                    // Normalize to [-180, 180]:
                                    angleDiff = (angleDiff + 180) % 360 - 180;

                                    this.angleToTagDegrees = angleDiff;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /** Get the best estimate of where we are on the field. */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the robot pose to a known location if we know exactly where we are.
     */
    public void resetPose(Pose2d newPose, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, newPose);
    }

    /** Update the field layout if needed. */
    public void setAprilTagFieldLayout(AprilTagFieldLayout newLayout) {
        aprilTagFieldLayout = newLayout;
        if (photonPoseEstimators != null) {
            for (PhotonPoseEstimator estimator : photonPoseEstimators) {
                if (estimator != null) {
                    estimator.setFieldTags(newLayout);
                }
            }
        }
    }

    /** Check if we are on the red alliance. */
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    @Override
    public void periodic() {
        // Nothing to do since DriveSubsystem calls update() for us.
    }

    /**
     * @return ID of the last-detected tag (or -1 if none)
     */
    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }

    /**
     * @return Which camera index last saw a tag (or -1 if none)
     */
    public int getLastDetectionCameraIndex() {
        return lastDetectionCameraIndex;
    }

    /**
     * @return Time at which the last detection happened
     */
    public double getLastDetectionTimestamp() {
        return lastDetectionTimestamp;
    }

    /**
     * @return Distance from the robot’s center to the tag in meters
     */
    public double getDistanceToTag() {
        return distanceToTag;
    }

    /**
     * @return Angle difference (deg) between robot heading and direction to tag
     *         (range ~[-180, 180])
     */
    public double getAngleToTagDegrees() {
        return angleToTagDegrees;
    }
}
