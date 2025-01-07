package frc.robot.subsystems;

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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    // Arrays hold multiple cameras and their estimators, so we can get data from more than one camera.
    private PhotonCamera[] photonCameras;
    private PhotonPoseEstimator[] photonPoseEstimators;

    // These constants describe how much uncertainty we think we have in our measurements.
    // Lower numbers mean we trust that measurement more.
    private static final double STATE_STD_DEV_POS = 0.03;
    private static final double STATE_STD_DEV_HEADING = 0.02;
    private static final double VISION_STD_DEV_POS = 0.9;
    private static final double VISION_STD_DEV_HEADING = 0.9;

    // If we have multiple cameras, we list their names here.
    //Add as many cameras we plan to use that season
    //TODO: FILL CAMERA NAMES IN PLEASE!!!
    private static final String[] CAMERA_NAMES = {"Arducam_1", "Arducam_3"/*"CameraFront", "CameraBack" */};

    // Each camera might be mounted differently on the robot.
    // These transforms describe the camera's position and orientation relative to the robot's center.
    private final Transform3d[] robotToCams = {
        //TODO: FILL CAMERA TRANSFORMS IN PLEASE!!!
        new Transform3d(/* front camera offset */),
        new Transform3d(/* front camera offset */)
        //Add as many cameras we plan to use that season
    };

    public PoseEstimatorSubsystem(Pose2d initialPose) {
        // Load the known positions of the AprilTags on the field for this season.
        // If this fails, we can't use vision to correct our pose.
        try {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (UncheckedIOException e) {
            e.printStackTrace();
            System.err.println("Failed to load AprilTagFieldLayout. We'll run without vision corrections.");
            aprilTagFieldLayout = null;
        }

        // Initialize arrays for multiple cameras
        photonCameras = new PhotonCamera[CAMERA_NAMES.length];
        photonPoseEstimators = new PhotonPoseEstimator[CAMERA_NAMES.length];

        if (aprilTagFieldLayout != null) {
            // For each camera, set it up along with a pose estimator that uses AprilTags.
            for (int i = 0; i < CAMERA_NAMES.length; i++) {
                photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
                photonPoseEstimators[i] = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCams[i]
                );
                

            }
            

        } else {
            // If no field layout, we still create cameras but won't be able to run the pose estimator from them.
            for (int i = 0; i < CAMERA_NAMES.length; i++) {
                photonCameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
            }
        }

        // Setup standard deviations (our guesses about how noisy our sensors are).
        var stateStdDevs = VecBuilder.fill(STATE_STD_DEV_POS, STATE_STD_DEV_POS, STATE_STD_DEV_HEADING);
        var visionStdDevs = VecBuilder.fill(VISION_STD_DEV_POS, VISION_STD_DEV_POS, VISION_STD_DEV_HEADING);

        // The SwerveDrivePoseEstimator is our big brains for combining sensor data into a single best guess pose.
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()},
            initialPose,
            stateStdDevs,
            visionStdDevs
        );
    }

    /**
     * Called periodically by the DriveSubsystem. We give it our current gyro angle and wheel positions,
     * and it updates its internal estimate of our position. Then we also look at all unread camera results
     * and see if we got any AprilTag detections that can improve our position guess.
     */
    public void update(Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        // First, update our pose from odometry (wheel + gyro)
        poseEstimator.update(gyroRotation, modulePositions);

        // If we have the field layout and pose estimators set up, we can try using vision.
        if (photonPoseEstimators != null && aprilTagFieldLayout != null) {
            for (int i = 0; i < photonCameras.length; i++) {
                PhotonCamera camera = photonCameras[i];
                PhotonPoseEstimator estimator = photonPoseEstimators[i];
                if (estimator == null) continue; // If something failed and we don't have an estimator.

                // getAllUnreadResults() gives us all camera frames since last time we called it.
                // We loop over them all in case multiple frames came in quickly.
                List<PhotonPipelineResult> results = camera.getAllUnreadResults();
                for (PhotonPipelineResult result : results) {
                    if (result.hasTargets()) {
                        // Run the vision estimate. If we see tags, we get a pose estimate.
                        Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
                        if (estimatedPose.isPresent()) {
                            // If we got a good pose estimate from vision, feed it into the Kalman filter.
                            EstimatedRobotPose camPose = estimatedPose.get();
                            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
                        }
                    }
                }
            }
        }
    }

    /**
     * Get the best estimate of where we are on the field.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the robot pose to a known location if we know exactly where we are.
     * For example, at the start of a match if we start in a known spot.
     */
    public void resetPose(Pose2d newPose, Rotation2d gyroRotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, newPose);
    }

    /**
     * If for some reason the AprilTag field layout changes or we load a new layout, we can update it here.
     */
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

    /**
     * Check if we are on the red alliance, which might matter for path following (mirroring).
     */
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    @Override
    public void periodic() {
        // Nothing to do here since DriveSubsystem calls update() for us.
    }
}
