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
import edu.wpi.first.math.geometry.Translation3d;
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

    private final SwerveDrivePoseEstimator poseEstimator;
    private final PhotonCamera[] photonCameras;

    // Standard deviations for state and vision measurements
    private static final double STATE_STD_DEV_POS = 0.03;
    private static final double STATE_STD_DEV_HEADING = 0.02;
    private static final double VISION_STD_DEV_POS = 0.9;
    private static final double VISION_STD_DEV_HEADING = 0.9;

    // Camera names from your PhotonVision config
    private static final String[] CAMERA_NAMES = {
        "Arducam_3"  // index 2 => right
    };

    /**
     * Robot-to-Camera transforms.
     * X = forward, Y = left, Z = up.
     * Rotation3d(roll, pitch, yaw) in radians.
     */
    private final Transform3d[] robotToCams = {
        // Index 1: front camera
        new Transform3d(
            new Translation3d(+0.267, 0.0, 0.165),
            new Rotation3d(0.0, 0.0, 0.0)
        )
    };

    // For AlignCommand
    private int lastDetectedTagId = -1;
    private int lastDetectionCameraIndex = -1;
    private double lastDetectionTimestamp = -1.0;

    // Distance/bearing to the tag center
    private double distanceToTag = Double.NaN;      
    private double bearingToTagDeg = Double.NaN;    
    private double lateralOffsetToTag = Double.NaN; 
    private double xOffsetToTag = Double.NaN;
    private double yOffsetToTag = Double.NaN;

    // Tag orientation difference relative to the robot
    private double tagOrientationErrorDeg = Double.NaN;

    public PoseEstimatorSubsystem(Pose2d initialPose) {
        photonCameras = new PhotonCamera[CAMERA_NAMES.length];
        for (int i = 0; i < CAMERA_NAMES.length; i++) {
            photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
        }

        var stateStdDevs = VecBuilder.fill(
            STATE_STD_DEV_POS, 
            STATE_STD_DEV_POS, 
            STATE_STD_DEV_HEADING
        );
        var visionStdDevs = VecBuilder.fill(
            VISION_STD_DEV_POS, 
            VISION_STD_DEV_POS, 
            VISION_STD_DEV_HEADING
        );

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

    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
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

            lastDetectedTagId = bestTarget.getFiducialId();
            lastDetectionCameraIndex = camIdx;
            lastDetectionTimestamp = result.getTimestampSeconds();

            Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
            if (cameraToTarget == null) {
                continue;
            }

            // Get robot-relative transform
            Transform3d robotToTarget = robotToCams[camIdx].inverse().plus(cameraToTarget);
            Translation3d translation = robotToTarget.getTranslation();
            Rotation3d rotation = robotToTarget.getRotation();

            // Calculate distance using X and Y components
            // double newDistance = Math.hypot(translation.getX(), translation.getY());
            double newDistance = Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2));
            
            // Calculate bearing to tag center (positive = tag is to the left)
            double bearingRadians = Math.atan2(translation.getY(), translation.getX());
            double newBearingDeg = Math.toDegrees(bearingRadians);
            
            // Calculate tag's orientation relative to robot
            // We use the X axis rotation which represents the tag's yaw relative to robot
            double tagYawRobotFrame = rotation.getX();  // in radians
            double newTagOrientationDeg = Math.toDegrees(tagYawRobotFrame);
            
            // Normalize to [-180, 180)
            newTagOrientationDeg = Math.IEEEremainder(newTagOrientationDeg + 180.0, 360.0) - 180.0;
            
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
        }
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
        return distanceToTag;
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
        return lateralOffsetToTag;
    }

    public double getXOffsetToTag() {
        return xOffsetToTag;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, newPose);
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
    }
}