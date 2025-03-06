package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class ApproachTagCommand extends Command {
    private final DriveSubsystem drive;
    private final double desiredDistance;
    private final PIDController distanceController;
    private final PIDController lateralController;
    private final PIDController rotationController;

    // Tolerances and speed limits
    private static final double DISTANCE_TOLERANCE_METERS = 0.015; // 1.5cm tolerance (example)
    private static final double LATERAL_TOLERANCE_METERS = 0.02; // 2cm
    private static final double ROTATION_TOLERANCE_DEG = 5.0; // degrees tolerance
    private static final double MAX_FORWARD_SPEED = 1.5; // m/s
    private static final double MAX_LATERAL_SPEED = 1.0; // m/s
    private static final double MAX_ROTATION_SPEED = 5.0; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds

    // Camera indices matching those in PoseEstimatorSubsystem
    private static final int LEFT_CAMERA_INDEX = 2;  // for intake mode when target is right-side
    private static final int RIGHT_CAMERA_INDEX = 1; // for intake mode when target is left-side

    private double lastTagTimestamp = 0;
    // Variables to lock in the initial lateral offset when the tag is first seen
    private boolean lateralOffsetInitialized = false;
    private double desiredLateralOffset = 0;
    private boolean isIntake = false;
    
    // Camera selection for both intake and non-intake modes.
    // In non-intake mode, front camera indices are used (0 for right side, 3 for left side)
    private int selectedCameraIndex = -1; // Default to no specific camera

    public ApproachTagCommand(DriveSubsystem drive, double desiredDistance, double desiredLateralOffset, boolean isIntake) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        this.desiredLateralOffset = desiredLateralOffset;
        this.isIntake = isIntake;
        addRequirements(drive);

        // PID for forward (distance) control
        distanceController = new PIDController(3.0, 0.0, 0.00);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);

        // PID for lateral offset correction
        lateralController = new PIDController(2.5, 0.0, 0.5);
        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);

        // PID for rotation to face the desired offset position
        rotationController = new PIDController(0.125, 0.0, 0.005);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180); // angle wrap-around
    }

    @Override
    public void initialize() {
        distanceController.reset();
        lateralController.reset();
        rotationController.reset();
        lateralOffsetInitialized = false;
        
        // Select the camera based on mode and desired lateral offset
        if (isIntake) {
            // In intake mode: use the dedicated intake camera selection.
            // Positive offset means target is to the right so use left intake camera.
            if (desiredLateralOffset > 0) {
                selectedCameraIndex = LEFT_CAMERA_INDEX;
                System.out.println("Intake mode: Using left camera (index " + LEFT_CAMERA_INDEX + ") for right-side approach");
            } else {
                selectedCameraIndex = RIGHT_CAMERA_INDEX;
                System.out.println("Intake mode: Using right camera (index " + RIGHT_CAMERA_INDEX + ") for left-side approach");
            }
            rotationController.setP(0.125);
        } else {
            // In non-intake mode: if there is a lateral offset, use a front camera.
            if (desiredLateralOffset > 0) {
                selectedCameraIndex = 0;  // Front camera for right-side approach
                System.out.println("Non-intake mode: Using front camera index 0 for right-side approach");
            } else if (desiredLateralOffset < 0) {
                selectedCameraIndex = 3;  // Front camera for left-side approach
                System.out.println("Non-intake mode: Using front camera index 3 for left-side approach");
            } else {
                selectedCameraIndex = -1; // No specific camera selected; use any available detection
                System.out.println("Non-intake mode: No lateral offset specified, using any available camera");
            }
            double curDistance = drive.getPoseEstimatorSubsystem().getDistanceToTag(selectedCameraIndex);

            if (curDistance > 1) {
                rotationController.setP(0.04);
            }
        }

        System.out.printf("ApproachTagCommand initialized - Target distance: %.2f m, Lateral offset: %.2f m, Intake mode: %b%n", 
            desiredDistance, desiredLateralOffset, isIntake);
    }

    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        boolean validTagDetection = false;
        
        // Use selected camera if one is set (applies to both intake and non-intake modes)
        if (selectedCameraIndex != -1) {
            int detectedTag = poseEstimator.getLastTagDetectedByCamera(selectedCameraIndex);
            double lastDetectionTime = poseEstimator.getLastCameraDetectionTimestamp(selectedCameraIndex);
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            
            if (detectedTag != -1 && (currentTime - lastDetectionTime) < LOST_TAG_TIMEOUT) {
                validTagDetection = true;
                lastTagTimestamp = lastDetectionTime;
            }
        } else {
            // Otherwise, use any camera detection available.
            if (poseEstimator.getLastDetectedTagId() != -1) {
                validTagDetection = true;
                lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            }
        }

        if (validTagDetection) {
            double currentDistance = poseEstimator.getDistanceToTag(selectedCameraIndex);
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag(selectedCameraIndex);
            double currentRotation = poseEstimator.getTagOrientationErrorDeg(selectedCameraIndex);

            // Compute corrections using PID controllers
            double forwardSpeed = -distanceController.calculate(currentDistance, desiredDistance);
            double lateralSpeed = -lateralController.calculate(currentLateralOffset, desiredLateralOffset);
            double rotationSpeed = -rotationController.calculate(currentRotation, 180);

            // Clamp speeds to maximum limits
            forwardSpeed = Math.min(Math.max(forwardSpeed, -MAX_FORWARD_SPEED), MAX_FORWARD_SPEED);
            lateralSpeed = Math.min(Math.max(lateralSpeed, -MAX_LATERAL_SPEED), MAX_LATERAL_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            // Drive the robot with computed speeds
            drive.driveRobotRelative(new ChassisSpeeds(forwardSpeed, lateralSpeed, rotationSpeed));
        } else {
            // No valid tag detected; stop the robot
            drive.driveRobotRelative(new ChassisSpeeds());
        }
    }

    @Override
    public boolean isFinished() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (currentTime - lastTagTimestamp > LOST_TAG_TIMEOUT) {
            System.out.println("ApproachTagCommand timeout - tag lost");
            return true;
        }

        boolean validTagDetection = false;
        double currentDistance = 0;
        double currentLateralOffset = 0;
        double currentRotation = 0;
        
        if (selectedCameraIndex != -1) {
            int detectedTag = poseEstimator.getLastTagDetectedByCamera(selectedCameraIndex);
            double lastDetectionTime = poseEstimator.getLastCameraDetectionTimestamp(selectedCameraIndex);
            
            if (detectedTag != -1 && (currentTime - lastDetectionTime) < LOST_TAG_TIMEOUT) {
                validTagDetection = true;
                currentDistance = poseEstimator.getDistanceToTag(selectedCameraIndex);
                currentLateralOffset = poseEstimator.getLateralOffsetToTag(selectedCameraIndex);
                currentRotation = poseEstimator.getTagOrientationErrorDeg(selectedCameraIndex);
            }
        } else {
            if (poseEstimator.getLastDetectedTagId() != -1) {
                validTagDetection = true;
                currentDistance = poseEstimator.getDistanceToTag(selectedCameraIndex);
                currentLateralOffset = poseEstimator.getLateralOffsetToTag(selectedCameraIndex);
                currentRotation = poseEstimator.getTagOrientationErrorDeg(selectedCameraIndex);
            }
        }

        if (validTagDetection) {
            // Check if the distance, lateral offset, and rotation are within tolerance.
            boolean distanceOk = Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS;
            boolean lateralOk = Math.abs(currentLateralOffset - desiredLateralOffset) < LATERAL_TOLERANCE_METERS;
            double rotationError = (currentRotation - 180);
            boolean rotationOk = Math.abs(rotationError) < ROTATION_TOLERANCE_DEG;

            return distanceOk && lateralOk && rotationOk;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("ApproachTagCommand ended. Interrupted: " + interrupted);
    }
}
