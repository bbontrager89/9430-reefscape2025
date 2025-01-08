package frc.robot.subsystems;

// Comment out WPILib AprilTag imports if not needed
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimatorSubsystem extends SubsystemBase {
    // Swerve Pose Estimator for wheel-odometry + gyro
    private final SwerveDrivePoseEstimator poseEstimator;

    // Comment out the layout and PhotonPoseEstimator
    // private AprilTagFieldLayout aprilTagFieldLayout;
    // private PhotonPoseEstimator[] photonPoseEstimators;

    // One camera array for multiple cameras (no layout usage)
    private PhotonCamera[] photonCameras;

    // Tuning constants for the pose estimator
    private static final double STATE_STD_DEV_POS = 0.03;
    private static final double STATE_STD_DEV_HEADING = 0.02;
    private static final double VISION_STD_DEV_POS = 0.9;
    private static final double VISION_STD_DEV_HEADING = 0.9;

    // Names must match what you configured in PhotonVision
    private static final String[] CAMERA_NAMES = {"Arducam_1", "Arducam_2", "Arducam_3"};

    // Camera-to-robot transforms (if you need them for your own distance/angle math)
    private final Transform3d[] robotToCams = {
        // Example transforms only; adjust to your actual camera positions
        new Transform3d(new Translation3d(-0.267, 0.0, 0.165), new Rotation3d(0, 0, Math.PI/2)),
        new Transform3d(new Translation3d(0.0,  0.267, 0.165), new Rotation3d(0, 0, 0.0)),
        new Transform3d(new Translation3d(0.267, 0.0, 0.165), new Rotation3d(0, 0, Math.PI/2))
    };

    // Fields used by AlignCommand, etc.
    private int lastDetectedTagId = -1;
    private int lastDetectionCameraIndex = -1;
    private double lastDetectionTimestamp = -1.0;
    private double distanceToTag = Double.NaN;
    private double angleToTagDegrees = Double.NaN;

    public PoseEstimatorSubsystem(Pose2d initialPose) {
        // -----------------------------------------------------------------
        // (A) Commented out: skipping AprilTagFieldLayout
        /*
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        } catch (UncheckedIOException e) {
            e.printStackTrace();
            System.err.println("Failed to load AprilTagFieldLayout. We'll run without it.");
            aprilTagFieldLayout = null;
        }
        */

        // (B) Create PhotonCameras (no layout usage)
        photonCameras = new PhotonCamera[CAMERA_NAMES.length];
        for (int i = 0; i < CAMERA_NAMES.length; i++) {
            photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
        }

        // (C) We won't create PhotonPoseEstimators since we aren't using field layout
        // photonPoseEstimators = new PhotonPoseEstimator[CAMERA_NAMES.length];
        // ...

        // (D) Create the SwerveDrivePoseEstimator for basic odometry
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
     * Called periodically by (e.g.) DriveSubsystem to update pose from wheels/gyro
     * and to update last seen tag distance/angle from cameras (but not using field layout).
     */
    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        // 1) Update from wheel/gyro
        poseEstimator.update(gyroRotation, modulePositions);

        // 2) Check each camera for new results, extract best target ID/dist/angle
        for (int camIdx = 0; camIdx < photonCameras.length; camIdx++) {
            PhotonCamera camera = photonCameras[camIdx];
            // Get all results since last check
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                if (!result.hasTargets()) {
                    continue;
                }
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                if (bestTarget == null) {
                    continue;
                }

                // Record ID, camera index, and timestamp from PhotonVision
                this.lastDetectedTagId = bestTarget.getFiducialId();
                this.lastDetectionCameraIndex = camIdx;
                // We can use result.getTimestampSeconds() from PhotonVision
                this.lastDetectionTimestamp = result.getTimestampSeconds();

                // Compute distance & angle from camera’s point of view
                var cameraToTag = bestTarget.getBestCameraToTarget(); // Transform3d
                double x = cameraToTag.getX();  // forward (meters)
                double y = cameraToTag.getY();  // left (meters)
                double distCam = Math.sqrt(x*x + y*y);
                double angleCamRad = Math.atan2(y, x);

                // If your camera is near the robot center, you can approximate:
                this.distanceToTag = distCam;
                this.angleToTagDegrees = Math.toDegrees(angleCamRad);

                // If you want to be more precise, you could transform these camera-based coords
                // to the robot center, but that requires more math with robotToCams[camIdx].
                // ...
            }
        }
    }

    /** Returns the current best pose estimate from odometry (no vision correction). */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Manually reset your odometry if you know your exact location. */
    public void resetPose(Pose2d newPose, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, newPose);
    }

    /** Check alliance color if needed. */
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    @Override
    public void periodic() {
        // Typically the "update(...)" is called from your DriveSubsystem instead.
    }

    // -------------------------------------------------------
    // Getters for AlignCommand or other usage:
    // -------------------------------------------------------
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

    public double getAngleToTagDegrees() {
        return angleToTagDegrees;
    }
}
