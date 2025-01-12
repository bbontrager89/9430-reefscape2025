package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * PoseEstimatorSubsystem:
 * - Tracks odometry via SwerveDrivePoseEstimator
 * - Reads camera data from PhotonVision
 * - Calculates AprilTag position in robot-centric coordinates:
 * distance (forward),
 * angle (bearing),
 * lateral offset,
 * forward offset.
 */
public class PoseEstimatorSubsystem extends SubsystemBase {

    // ----------------------------------------------------
    // Swerve Pose Estimator for wheel-odometry + gyro
    // ----------------------------------------------------
    private final SwerveDrivePoseEstimator poseEstimator;

    // Cameras
    private final PhotonCamera[] photonCameras;

    // Tuning constants
    private static final double STATE_STD_DEV_POS = 0.03; // Position std dev
    private static final double STATE_STD_DEV_HEADING = 0.02; // Heading std dev
    private static final double VISION_STD_DEV_POS = 0.9; // Vision measurement std dev
    private static final double VISION_STD_DEV_HEADING = 0.9;

    // Camera names from your PhotonVision config
    private static final String[] CAMERA_NAMES = {
            "Arducam_1", // index 0 => left
            "Arducam_2", // index 1 => front
            "Arducam_3" // index 2 => right
    };

    /**
     * Robot-to-Camera transforms.
     * Adjust based on your actual camera mounting positions/orientations.
     *
     * X = forward, Y = left, Z = up
     * Rotation3d( pitch, yaw, roll ) in radians
     * - Usually we only rotate about the Z axis if the camera is angled left/right
     * - Or rotate about Y if tilted up/down
     */
    private final Transform3d[] robotToCams = {
            // Index 0: left
            new Transform3d(
                    new Translation3d(0.0, +0.267, 0.165), // left side
                    new Rotation3d(0.0, 2.00713, Math.PI / 2.0) // facing forward from left side?
            ),
            // Index 1: front
            new Transform3d(
                    new Translation3d(+0.267, 0.0, 0.165), // in front
                    new Rotation3d(0.0, 0.0, 0.0) // facing forward
            ),
            // Index 2: right
            new Transform3d(
                    new Translation3d(0.0, -0.267, 0.165),
                    new Rotation3d(0.0, 2.00713, -Math.PI / 2.0))
    };

    // ----------------------------------------------------
    // Fields used by AlignCommand (or other commands)
    // ----------------------------------------------------
    private int lastDetectedTagId = -1;
    private int lastDetectionCameraIndex = -1;
    private double lastDetectionTimestamp = -1.0;

    // Robot-centric measurements
    private double distanceToTag = Double.NaN; // Hypotenuse in XY plane
    private double angleToTagDegrees = Double.NaN; // Bearing in degrees
    private double lateralOffsetToTag = Double.NaN;
    private double xOffsetToTag = Double.NaN;

    public PoseEstimatorSubsystem(Pose2d initialPose) {
        photonCameras = new PhotonCamera[CAMERA_NAMES.length];
        for (int i = 0; i < CAMERA_NAMES.length; i++) {
            photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
        }

        var stateStdDevs = VecBuilder.fill(
                STATE_STD_DEV_POS,
                STATE_STD_DEV_POS,
                STATE_STD_DEV_HEADING);
        var visionStdDevs = VecBuilder.fill(
                VISION_STD_DEV_POS,
                VISION_STD_DEV_POS,
                VISION_STD_DEV_HEADING);

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
                visionStdDevs);
    }

    /**
     * Call this periodically (in Robot or DriveSubsystem) to update odometry and
     * vision.
     * 
     * @param gyroRotation    Current heading from gyro
     * @param modulePositions Current swerve module wheel distances/angles
     */
    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        // 1) Update with wheel/gyro data
        poseEstimator.update(gyroRotation, modulePositions);

        // 2) Process any new camera results
        for (int camIdx = 0; camIdx < photonCameras.length; camIdx++) {
            PhotonCamera camera = photonCameras[camIdx];
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            for (PhotonPipelineResult result : results) {
                if (!result.hasTargets()) {
                    continue;
                }

                PhotonTrackedTarget bestTarget = result.getBestTarget();
                if (bestTarget == null) {
                    continue;
                }

                // Record detection info
                lastDetectedTagId = bestTarget.getFiducialId();
                lastDetectionCameraIndex = camIdx;
                lastDetectionTimestamp = result.getTimestampSeconds();

                // Get raw yaw and distance data from PhotonVision
                double rawYaw = bestTarget.getYaw(); // degrees from camera's perspective

                // For side cameras, we need to adjust the yaw based on camera mounting
                if (camIdx == 0) { // Left camera
                    angleToTagDegrees = rawYaw + 90; // Add 90 since camera is rotated 90 degrees
                } else if (camIdx == 2) { // Right camera
                    angleToTagDegrees = rawYaw - 90; // Subtract 90 since camera is rotated -90 degrees
                } else { // Front camera
                    angleToTagDegrees = rawYaw; // Use raw yaw directly
                }

                // Get distance from PhotonVision's pose estimate
                Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
                Transform3d robotToTag = robotToCams[camIdx].plus(cameraToTag);

                // Extract translation components
                Translation3d t = robotToTag.getTranslation();
                double rx = t.getX();
                double ry = t.getY();
                double rz = t.getZ();

                // Convert camera-relative coordinates to robot-relative coordinates
                double robotX, robotY;
                if (camIdx == 0) { // Left camera
                    robotX = rx; // Camera's forward is already robot's left
                    robotY = ry; // Camera's left is already robot's forward
                } else if (camIdx == 2) { // Right camera
                    robotX = rx; // Camera's forward is robot's right
                    robotY = -ry; // Camera's right is robot's backward
                } else { // Front camera
                    robotX = rx; // Already in robot coordinates
                    robotY = ry;
                }

                // Store in robot-relative coordinates
                distanceToTag = Math.hypot(robotX, robotY);
                lateralOffsetToTag = robotY;
                xOffsetToTag = robotX;

            }
        }
    }

    // ----------------------------------------------------
    // Accessors used by AlignCommand or other code
    // ----------------------------------------------------
    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }

    public int getLastDetectionCameraIndex() {
        return lastDetectionCameraIndex;
    }

    public double getLastDetectionTimestamp() {
        return lastDetectionTimestamp;
    }

    /** Distance from robot center to the tag, in meters (XY plane). */
    public double getDistanceToTag() {
        return distanceToTag;
    }

    /**
     * Angle from the robot forward axis to the tag, in degrees.
     * + => tag is to the left, - => tag is to the right, 0 => directly ahead
     */
    public double getAngleToTagDegrees() {
        return angleToTagDegrees;
    }

    /**
     * Lateral side-to-side offset from robot center to the tag, + => left, - =>
     * right.
     */
    public double getLateralOffsetToTag() {
        return lateralOffsetToTag;
    }

    /**
     * Forward/back offset from robot center to the tag, + => in front, - => behind.
     */
    public double getXOffsetToTag() {
        return xOffsetToTag;
    }

    /** Current best estimate of the robot pose on the field. */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Allows you to reset the estimated pose (e.g., after auto start).
     */
    public void resetPose(Pose2d newPose, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, newPose);
    }

    /** Utility to check alliance color. */
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
    }
}
