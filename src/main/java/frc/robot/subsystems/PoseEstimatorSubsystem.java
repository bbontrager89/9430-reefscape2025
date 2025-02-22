package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

/**
 * PoseEstimatorSubsystem:
 * - Tracks odometry via SwerveDrivePoseEstimator.
 * - Reads camera data from PhotonVision.
 * - Calculates AprilTag position + orientation in robot-centric coordinates.
 */
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
        "Arducam_3",  // index 0 => front
        "Arducam_1",  // index 1 => left
        "Arducam_2"   // index 2 => right
    };
    
    // Camera indices for side cameras used in intake operations
    private static final int LEFT_CAMERA_INDEX = 2;
    private static final int RIGHT_CAMERA_INDEX = 1;

    /**
     * Robot-to-Camera transforms.
     * X = forward, Y = left, Z = up.
     * Rotation3d(roll, pitch, yaw) in radians.
     */
    private final Transform3d[] robotToCams = {
        // Front camera
        new Transform3d(
            new Translation3d(0.279, 0.0, 0.1096),
            new Rotation3d(0.0, 1.8326, 0.0)
        ),
        
       
    // Left camera (index 1) - now with positive Y offset
    new Transform3d(
        new Translation3d(-0.2018, 0.2616, 0.0559),
        new Rotation3d(1.3713, 1.1592, 1.1665)  // Adjust yaw as needed
    ),
    
    // Right camera (index 2) - now with negative Y offset
    new Transform3d(
        new Translation3d(-0.2018, -0.2612, 0.0561),
        new Rotation3d(1.3713, 1.15, -1.1665)   // Adjust yaw as needed
    )
    };

    // For AlignCommand
    private int lastDetectedTagId = -1;
    private int lastDetectionCameraIndex = -1;
    private double lastDetectionTimestamp = -1.0;

    // Track which cameras have detected tags
    private boolean[] cameraHasDetectedTag = new boolean[CAMERA_NAMES.length];
    private double[] lastCameraDetectionTimestamps = new double[CAMERA_NAMES.length];
    private int[] lastTagsDetectedByCamera = new int[CAMERA_NAMES.length];

    // Distance/bearing to the tag center in robot coordinate frame
    private double distanceToTag = Double.NaN;      
    private double bearingToTagDeg = Double.NaN;    
    private double lateralOffsetToTag = Double.NaN; 
    private double xOffsetToTag = Double.NaN;
    private double yOffsetToTag = Double.NaN;

    // Tag orientation difference relative to the robot
    private double tagOrientationErrorDeg = Double.NaN;
    
    // Robot heading at time of last detection
    private Rotation2d lastDetectionHeading = new Rotation2d();
    
    // Robot's current heading from gyro
    private Rotation2d currentHeading = new Rotation2d();

    public PoseEstimatorSubsystem(Pose2d initialPose) {
        photonCameras = new PhotonCamera[CAMERA_NAMES.length];
        for (int i = 0; i < CAMERA_NAMES.length; i++) {
            photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
            cameraHasDetectedTag[i] = false;
            lastCameraDetectionTimestamps[i] = -1.0;
            lastTagsDetectedByCamera[i] = -1;
        }

        poseEstimator = new SwerveDriveOdometry(
            Constants.DriveConstants.kDriveKinematics,
            initialPose.getRotation(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            }
        );
    }

    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        // Store current heading for use in coordinate transformations
        this.currentHeading = gyroRotation;
        
        // Update pose estimator with odometry
        poseEstimator.update(gyroRotation, modulePositions);
        double currentTime = Timer.getFPGATimestamp();

        for (int camIdx = 0; camIdx < photonCameras.length; camIdx++) {
            PhotonPipelineResult result = photonCameras[camIdx].getLatestResult();
            if (!result.hasTargets()) {
                continue;
            }
            
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            if (bestTarget == null) {
                continue;
            }

            // Update detection metadata
            lastDetectedTagId = bestTarget.getFiducialId();
            lastDetectionCameraIndex = camIdx;
            lastDetectionTimestamp = result.getTimestampSeconds();
            lastDetectionHeading = gyroRotation;

            // Update per-camera tracking
            cameraHasDetectedTag[camIdx] = true;
            lastCameraDetectionTimestamps[camIdx] = result.getTimestampSeconds();
            lastTagsDetectedByCamera[camIdx] = bestTarget.getFiducialId();

            // Update dashboard with camera-specific info
            SmartDashboard.putBoolean("Camera " + camIdx + " Has Detection", true);
            SmartDashboard.putNumber("Camera " + camIdx + " Last Detection Time", lastCameraDetectionTimestamps[camIdx]);
            SmartDashboard.putNumber("Camera " + camIdx + " Last Tag ID", lastTagsDetectedByCamera[camIdx]);

            Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
            if (cameraToTarget == null) {
                continue;
            }

            // Get robot-relative transform
            Transform3d robotToTarget = robotToCams[camIdx].plus(cameraToTarget);
            Translation3d translation = robotToTarget.getTranslation();
            Rotation3d rotation = robotToTarget.getRotation();

            // Calculate distance using X and Y components in robot frame
            double newDistance = Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2));
            
            // Calculate bearing to tag center (positive = tag is to the left)
            double bearingRadians = Math.atan2(translation.getY(), translation.getX());
            double newBearingDeg = Math.toDegrees(bearingRadians);
            
            // Calculate the tag's orientation relative to the robot
            double tagYawRobotFrame = rotation.getZ();  // in radians
            double newTagOrientationDeg = Math.toDegrees(tagYawRobotFrame);
            
            // Normalize to [-180, 180)
            newTagOrientationDeg = normalizeDegrees(newTagOrientationDeg);
            
            // Calculate the lateral offset (perpendicular distance to tag)
            double newLateralOffset = newDistance * Math.sin(bearingRadians);

            // Store updated values
            distanceToTag = newDistance;
            bearingToTagDeg = newBearingDeg;
            lateralOffsetToTag = newLateralOffset;
            xOffsetToTag = translation.getX();
            yOffsetToTag = translation.getY();
            tagOrientationErrorDeg = newTagOrientationDeg;

            // Update the dashboard with the latest values
            SmartDashboard.putNumber("Tag Bearing (Deg)", bearingToTagDeg);
            SmartDashboard.putNumber("Tag Orientation Error (Deg)", tagOrientationErrorDeg);
            SmartDashboard.putNumber("DistanceToTag", distanceToTag);
            SmartDashboard.putNumber("LateralOffsetToTag", lateralOffsetToTag);
            SmartDashboard.putNumber("xOffsetToTag", xOffsetToTag);
            SmartDashboard.putNumber("yOffsetToTag", yOffsetToTag);
            SmartDashboard.putNumber("Robot Heading", gyroRotation.getDegrees());
        }
    }
    public static double normalizeDegrees(double degrees) {
        degrees = degrees % 360;
        return degrees < 0 ? degrees + 360 : degrees;
    }
    /**
     * Checks if any side camera (left or right) has detected a tag for intake operations
     * @return true if either side camera has detected a tag
     */
    public boolean hasSideCameraDetection() {
        double currentTime = Timer.getFPGATimestamp();
        double leftTimestamp = lastCameraDetectionTimestamps[LEFT_CAMERA_INDEX];
        double rightTimestamp = lastCameraDetectionTimestamps[RIGHT_CAMERA_INDEX];
        
        // Check if either side camera has a recent detection
        boolean hasLeftDetection = leftTimestamp != -1.0 && (currentTime - leftTimestamp) < 1.0;
        boolean hasRightDetection = rightTimestamp != -1.0 && (currentTime - rightTimestamp) < 1.0;
        
        return hasLeftDetection || hasRightDetection;
    }

    /**
     * Gets the most recent side camera detection information
     * @return The index of the side camera with the most recent detection, or -1 if no detection
     */
    public int getLastSideCameraDetection() {
        double leftTimestamp = lastCameraDetectionTimestamps[LEFT_CAMERA_INDEX];
        double rightTimestamp = lastCameraDetectionTimestamps[RIGHT_CAMERA_INDEX];
        
        if (leftTimestamp == -1.0 && rightTimestamp == -1.0) {
            return -1;
        }
        
        return (leftTimestamp > rightTimestamp) ? LEFT_CAMERA_INDEX : RIGHT_CAMERA_INDEX;
    }

    /**
     * Gets whether a specific camera has ever detected a tag
     * @param cameraIndex The index of the camera to check
     * @return true if the camera has detected a tag
     */
    public boolean hasCameraDetectedTag(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return false;
        }
        return cameraHasDetectedTag[cameraIndex];
    }

    /**
     * Gets the timestamp of the last detection for a specific camera
     * @param cameraIndex The index of the camera to check
     * @return The timestamp of the last detection, or -1 if no detection
     */
    public double getLastCameraDetectionTimestamp(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return -1.0;
        }
        return lastCameraDetectionTimestamps[cameraIndex];
    }

    /**
     * Gets the ID of the last tag detected by a specific camera
     * @param cameraIndex The index of the camera to check
     * @return The ID of the last detected tag, or -1 if no detection
     */
    public int getLastTagDetectedByCamera(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return -1;
        }
        return lastTagsDetectedByCamera[cameraIndex];
    }

    /**
     * Gets the name of the camera at the specified index
     * @param cameraIndex The index of the camera
     * @return The name of the camera, or null if invalid index
     */
    public String getCameraName(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= CAMERA_NAMES.length) {
            return null;
        }
        return CAMERA_NAMES[cameraIndex];
    }

    /**
     * Transforms robot-relative coordinates to field-relative coordinates.
     * @param robotRelative The point in robot-relative coordinates.
     * @return The point in field-relative coordinates.
     */
    public Translation2d robotToFieldCoordinates(Translation2d robotRelative) {
        return robotRelative.rotateBy(currentHeading);
    }

    /**
     * Transforms field-relative coordinates to robot-relative coordinates.
     * @param fieldRelative The point in field-relative coordinates.
     * @return The point in robot-relative coordinates.
     */
    public Translation2d fieldToRobotCoordinates(Translation2d fieldRelative) {
        return fieldRelative.rotateBy(currentHeading.unaryMinus());
    }

    /**
     * Gets the lateral offset to the tag, adjusting for the robot's current orientation.
     * This ensures the lateral offset is consistent regardless of the robot's heading.
     * @return The adjusted lateral offset in meters.
     */
    public double getAdjustedLateralOffsetToTag() {
        if (Double.isNaN(lateralOffsetToTag) || lastDetectedTagId == -1) {
            return Double.NaN;
        }
        
        // Calculate heading change since last detection
        Rotation2d headingChange = currentHeading.minus(lastDetectionHeading);
        
        // Create a tag position in robot coordinates at time of detection
        Translation2d tagPositionAtDetection = new Translation2d(xOffsetToTag, yOffsetToTag);
        
        // Rotate the tag position based on how much the robot has turned since detection
        Translation2d adjustedTagPosition = tagPositionAtDetection.rotateBy(headingChange);
        
        // Return the Y component which is the lateral offset
        return adjustedTagPosition.getY();
    }

    // Accessors
    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }

    public int getLastDetectionCameraIndex() {
        return lastDetectionCameraIndex;
    }

    public double getLastDetectionTimestamp() {
        return lastDetectionTimestamp;
    }

    public double getDistanceToTag() {
        return distanceToTag;
    }
    
    public double getYOffsetToTag() {
        return yOffsetToTag;
    }

    /** Angle from robot's forward axis to the tag's position. Positive means tag is to the left. */
    public double getBearingToTagDeg() {
        return bearingToTagDeg;
    }

    /** The tag's orientation relative to the robot. 0° means tag and robot are parallel. */
    public double getTagOrientationErrorDeg() {
        return tagOrientationErrorDeg;
    }

    public double getLateralOffsetToTag() {
        // Use the adjusted method to handle rotation changes
        double adjusted = getAdjustedLateralOffsetToTag();
        if (!Double.isNaN(adjusted)) {
            return adjusted;
        }
        return lateralOffsetToTag;
    }

    public double getXOffsetToTag() {
        return xOffsetToTag;
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
}