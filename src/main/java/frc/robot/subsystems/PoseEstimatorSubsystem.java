package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimatorSubsystem extends SubsystemBase {

    private final SwerveDriveOdometry poseEstimator;
    private final PhotonCamera[] photonCameras;

    // Standard deviations for state and vision measurements
    private static final double STATE_STD_DEV_POS = 0.03;
    private static final double STATE_STD_DEV_HEADING = 0.02;
    private static final double VISION_STD_DEV_POS = 0.9;
    private static final double VISION_STD_DEV_HEADING = 0.9;

    // Camera names from your PhotonVision config
    private static final String[] CAMERA_NAMES = {
            "Arducam_5", // index 0: front left (or front)
            "Arducam_7", // index 1: right
            "Arducam 2", // index 2: left (used for intake)
            "Arducam_6" // index 3: front right (or front)
    };

    // Camera indices for side cameras used in intake operations
    private static final int LEFT_CAMERA_INDEX = 1;
    private static final int RIGHT_CAMERA_INDEX = 2;

    /**
     * Robot-to-Camera transforms.
     * X = forward, Y = left, Z = up.
     * Rotation3d(roll, pitch, yaw) in radians.
     */
    private final Transform3d[] robotToCams = {
            // Front Left camera (index 0)
            new Transform3d(
                    new Translation3d(0.288, 0.1397, 0.119),
                    new Rotation3d(0.0, -0.2617, 0.0)),

            // Right camera (index 1)
            new Transform3d(
                    new Translation3d(-0.196, -0.26, 0.140),
                    new Rotation3d(0, -1.16, -0.434) // Adjust yaw as needed
            ),

            // Left camera (index 2)
            new Transform3d(
                    new Translation3d(-0.196, 0.26, 0.140),
                    new Rotation3d(0, -1.16, 0.434) // Adjust yaw as needed
            ),
            // Front Right Camera (index 3)
            new Transform3d(
                    new Translation3d(0.288, -0.1397, 0.119),
                    new Rotation3d(0.0, -0.2617, 0.0)),
    };

    // A helper class to hold detection information per camera.
    private static class CameraDetection {
        int tagId;
        double detectionTimestamp;
        Rotation2d detectionHeading;
        double distanceToTag;
        double bearingToTagDeg;
        double lateralOffsetToTag;
        double xOffsetToTag;
        double yOffsetToTag;
        double tagOrientationErrorDeg;
        int cameraIndex;

        public CameraDetection(int tagId, double detectionTimestamp, Rotation2d detectionHeading,
                double distanceToTag, double bearingToTagDeg, double lateralOffsetToTag,
                double xOffsetToTag, double yOffsetToTag, double tagOrientationErrorDeg,
                int cameraIndex) {
            this.tagId = tagId;
            this.detectionTimestamp = detectionTimestamp;
            this.detectionHeading = detectionHeading;
            this.distanceToTag = distanceToTag;
            this.bearingToTagDeg = bearingToTagDeg;
            this.lateralOffsetToTag = lateralOffsetToTag;
            this.xOffsetToTag = xOffsetToTag;
            this.yOffsetToTag = yOffsetToTag;
            this.tagOrientationErrorDeg = tagOrientationErrorDeg;
            this.cameraIndex = cameraIndex;
        }
    }

    // Array storing detection info for each camera
    private final CameraDetection[] cameraDetections = new CameraDetection[CAMERA_NAMES.length];

    // Robot's current heading from gyro
    private Rotation2d currentHeading = new Rotation2d();

    public PoseEstimatorSubsystem(Pose2d initialPose) {
        photonCameras = new PhotonCamera[CAMERA_NAMES.length];
        for (int i = 0; i < CAMERA_NAMES.length; i++) {
            photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
            cameraDetections[i] = null;
        }

        poseEstimator = new SwerveDriveOdometry(
                Constants.DriveConstants.kDriveKinematics,
                initialPose.getRotation(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                });
    }

    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        // Store current heading for use in coordinate transformations
        this.currentHeading = gyroRotation;

        // Update pose estimator with odometry
        poseEstimator.update(gyroRotation, modulePositions);
        double currentTime = Timer.getFPGATimestamp();

        // Loop through each camera and update its detection info independently.
        for (int camIdx = 0; camIdx < photonCameras.length; camIdx++) {
            PhotonPipelineResult result = photonCameras[camIdx].getLatestResult();
            if (!result.hasTargets()) {
                cameraDetections[camIdx] = null;
                SmartDashboard.putBoolean("Camera " + camIdx + " Has Detection", false);
                continue;
            }

            PhotonTrackedTarget bestTarget = result.getBestTarget();
            if (bestTarget == null) {
                cameraDetections[camIdx] = null;
                SmartDashboard.putBoolean("Camera " + camIdx + " Has Detection", false);
                continue;
            }

            Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
            if (cameraToTarget == null) {
                cameraDetections[camIdx] = null;
                SmartDashboard.putBoolean("Camera " + camIdx + " Has Detection", false);
                continue;
            }

            // Get robot-relative transform for this camera.
            Transform3d robotToTarget = robotToCams[camIdx].plus(cameraToTarget);
            Translation3d translation = robotToTarget.getTranslation();
            Rotation3d rotation = robotToTarget.getRotation();

            // Create and store detection info for this camera.
            CameraDetection detection = new CameraDetection(
                    bestTarget.getFiducialId(),
                    result.getTimestampSeconds(),
                    gyroRotation,
                    translation.getMeasureX().in(Meter),
                    rotation.getMeasureZ().in(Degree),
                    translation.getMeasureY().in(Meter),
                    translation.getX(),
                    translation.getY(),
                    rotation.getMeasureZ().in(Degree),
                    camIdx);
            cameraDetections[camIdx] = detection;

            // Update dashboard with camera-specific info.
            SmartDashboard.putBoolean("Camera " + camIdx + " Has Detection", true);
            SmartDashboard.putNumber("Camera " + camIdx + " Last Detection Time", detection.detectionTimestamp);
            SmartDashboard.putNumber("Camera " + camIdx + " Last Tag ID", detection.tagId);
            SmartDashboard.putNumber("Camera " + camIdx + " DistanceToTag", detection.distanceToTag);
            SmartDashboard.putNumber("Camera " + camIdx + " Tag Bearing (Deg)", detection.bearingToTagDeg);
            SmartDashboard.putNumber("Camera " + camIdx + " Tag Orientation Error (Deg)",
                    detection.tagOrientationErrorDeg);
            SmartDashboard.putNumber("Camera " + camIdx + " LateralOffsetToTag", detection.lateralOffsetToTag);
            SmartDashboard.putNumber("Camera " + camIdx + " xOffsetToTag", detection.xOffsetToTag);
            SmartDashboard.putNumber("Camera " + camIdx + " yOffsetToTag", detection.yOffsetToTag);
        }
    }

    // Helper method to get the most recent detection across all cameras.
    private CameraDetection getMostRecentDetection() {
        CameraDetection mostRecent = null;
        for (CameraDetection detection : cameraDetections) {
            if (detection != null) {
                if (mostRecent == null || detection.detectionTimestamp > mostRecent.detectionTimestamp) {
                    mostRecent = detection;
                }
            }
        }
        return mostRecent;
    }

    // Global getters (aggregated using the most recent detection)
    public int getLastDetectedTagId() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.tagId : -1;
    }

    public int getLastDetectionCameraIndex() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.cameraIndex : -1;
    }

    public double getLastDetectionTimestamp() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.detectionTimestamp : -1.0;
    }

    /**
     * Gets the distance to the tag for a specified camera index.
     * 
     * @param cameraIndex The index of the camera.
     * @return The distance to the tag in meters, or NaN if no detection exists.
     */
    public double getDistanceToTag(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return Double.NaN;
        }
        CameraDetection detection = cameraDetections[cameraIndex];
        return detection != null ? detection.distanceToTag : Double.NaN;
    }

    /**
     * Gets the lateral offset to the tag for a specified camera index.
     * 
     * @param cameraIndex The index of the camera.
     * @return The lateral offset in meters, or NaN if no detection exists.
     */
    public double getLateralOffsetToTag(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return Double.NaN;
        }
        CameraDetection detection = cameraDetections[cameraIndex];
        return detection != null ? detection.lateralOffsetToTag : Double.NaN;
    }

    /**
     * Angle from robot's forward axis to the tag's position. Positive means tag is
     * to the left.
     */
    public double getBearingToTagDeg() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.bearingToTagDeg : Double.NaN;
    }

    /**
     * The tag's orientation relative to the robot. 0° means tag and robot are
     * parallel.
     */
    public double getTagOrientationErrorDeg() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.tagOrientationErrorDeg : Double.NaN;
    }

    public double getTagOrientationErrorDeg(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return Double.NaN;
        }
        CameraDetection detection = cameraDetections[cameraIndex];
        return detection != null ? detection.tagOrientationErrorDeg : Double.NaN;
    }

    /**
     * Gets the X offset to the tag from the most recent detection.
     * 
     * @return The X offset in meters.
     */
    public double getXOffsetToTag() {
        CameraDetection detection = getMostRecentDetection();
        return detection != null ? detection.xOffsetToTag : Double.NaN;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getPoseMeters();
    }

    public void resetPose(Pose2d newPose, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, newPose);
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
    }

    /**
     * Checks if any side camera (left or right) has detected a tag for intake
     * operations.
     * 
     * @return true if either side camera has a recent detection.
     */
    public boolean hasSideCameraDetection() {
        double currentTime = Timer.getFPGATimestamp();
        boolean hasLeftDetection = cameraDetections[LEFT_CAMERA_INDEX] != null &&
                (currentTime - cameraDetections[LEFT_CAMERA_INDEX].detectionTimestamp) < 1.0;
        boolean hasRightDetection = cameraDetections[RIGHT_CAMERA_INDEX] != null &&
                (currentTime - cameraDetections[RIGHT_CAMERA_INDEX].detectionTimestamp) < 1.0;
        return hasLeftDetection || hasRightDetection;
    }

    /**
     * Checks if any front camera (indices 0 or 3) has detected a tag.
     * 
     * @return true if either front camera has a recent detection.
     */
    public boolean hasFrontCameraDetection() {
        double currentTime = Timer.getFPGATimestamp();
        boolean hasLeftDetection = cameraDetections[0] != null &&
                (currentTime - cameraDetections[0].detectionTimestamp) < 1.0;
        boolean hasRightDetection = cameraDetections[3] != null &&
                (currentTime - cameraDetections[3].detectionTimestamp) < 1.0;
        return hasLeftDetection || hasRightDetection;
    }

    /**
     * Gets the most recent side camera detection information.
     * 
     * @return The index of the side camera with the most recent detection, or -1 if
     *         no detection.
     */
    public int getLastSideCameraDetection() {
        CameraDetection leftDetection = cameraDetections[LEFT_CAMERA_INDEX];
        CameraDetection rightDetection = cameraDetections[RIGHT_CAMERA_INDEX];

        if (leftDetection == null && rightDetection == null) {
            return -1;
        }

        if (leftDetection != null && rightDetection != null) {
            return (leftDetection.detectionTimestamp > rightDetection.detectionTimestamp) ? LEFT_CAMERA_INDEX
                    : RIGHT_CAMERA_INDEX;
        }

        return leftDetection != null ? LEFT_CAMERA_INDEX : RIGHT_CAMERA_INDEX;
    }

    /**
     * Checks whether a specific camera has detected a tag recently.
     * 
     * @param cameraIndex The index of the camera to check.
     * @return true if the camera has a detection within the last 0.6 seconds.
     */
    public boolean hasCameraDetectedTag(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return false;
        }
        double currentTime = Timer.getFPGATimestamp();
        CameraDetection detection = cameraDetections[cameraIndex];
        return detection != null && (currentTime - detection.detectionTimestamp) < 0.6;
    }

    /**
     * Gets the timestamp of the last detection for a specific camera.
     * 
     * @param cameraIndex The index of the camera.
     * @return The timestamp of the last detection, or -1 if none.
     */
    public double getLastCameraDetectionTimestamp(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return -1.0;
        }
        CameraDetection detection = cameraDetections[cameraIndex];
        return detection != null ? detection.detectionTimestamp : -1.0;
    }

    /**
     * Gets the ID of the last tag detected by a specific camera.
     * 
     * @param cameraIndex The index of the camera.
     * @return The tag ID, or -1 if none.
     */
    public int getLastTagDetectedByCamera(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return -1;
        }
        CameraDetection detection = cameraDetections[cameraIndex];
        return detection != null ? detection.tagId : -1;
    }

    /**
     * Gets the name of the camera at the specified index.
     * 
     * @param cameraIndex The index of the camera.
     * @return The camera name, or null if invalid.
     */
    public String getCameraName(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return null;
        }
        return CAMERA_NAMES[cameraIndex];
    }

    /**
     * Transforms robot-relative coordinates to field-relative coordinates.
     * 
     * @param robotRelative The point in robot-relative coordinates.
     * @return The point in field-relative coordinates.
     */
    public Translation2d robotToFieldCoordinates(Translation2d robotRelative) {
        return robotRelative.rotateBy(currentHeading);
    }

    /**
     * Transforms field-relative coordinates to robot-relative coordinates.
     * 
     * @param fieldRelative The point in field-relative coordinates.
     * @return The point in robot-relative coordinates.
     */
    public Translation2d fieldToRobotCoordinates(Translation2d fieldRelative) {
        return fieldRelative.rotateBy(currentHeading.unaryMinus());
    }

    /**
     * Gets the lateral offset to the tag, adjusted for the robot's current
     * orientation.
     * This ensures the lateral offset is consistent regardless of the robot's
     * heading.
     * 
     * @return The adjusted lateral offset in meters.
     */
    public double getAdjustedLateralOffsetToTag() {
        CameraDetection detection = getMostRecentDetection();
        if (detection == null) {
            return Double.NaN;
        }

        // Calculate the heading change since the detection was recorded.
        Rotation2d headingChange = currentHeading.minus(detection.detectionHeading);

        // Create a position vector for the tag as seen at the time of detection.
        Translation2d tagPositionAtDetection = new Translation2d(detection.xOffsetToTag, detection.yOffsetToTag);

        // Rotate the vector by the heading change.
        Translation2d adjustedTagPosition = tagPositionAtDetection.rotateBy(headingChange);

        // Return the Y component (lateral offset).
        return adjustedTagPosition.getY();
    }

    public static double normalizeDegrees(double degrees) {
        degrees = degrees % 360;
        return degrees < 0 ? degrees + 360 : degrees;
    }
}
