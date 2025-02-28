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
    private static final double DISTANCE_TOLERANCE_METERS = 0.015; // 3cm
    private static final double LATERAL_TOLERANCE_METERS = 0.015; // 3cm
    private static final double ROTATION_TOLERANCE_DEG = 2.0; // degrees tolerance for rotation error
    private static final double MAX_FORWARD_SPEED = 0.5; // m/s
    private static final double MAX_LATERAL_SPEED = 1.0; // m/s
    private static final double MAX_ROTATION_SPEED = 0.3; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds

    // Camera indices matching those in PoseEstimatorSubsystem
    private static final int LEFT_CAMERA_INDEX = 2; // Matches index in PoseEstimatorSubsystem
    private static final int RIGHT_CAMERA_INDEX = 1; // Matches index in PoseEstimatorSubsystem

    private double lastTagTimestamp = 0;
    // Variables to lock in the initial lateral offset when the tag is first seen
    private boolean lateralOffsetInitialized = false;
    private double desiredLateralOffset = 0;
    private boolean isIntake = false;
    
    // Camera selection for intake mode
    private int selectedCameraIndex = -1; // Default to no specific camera

    public ApproachTagCommand(DriveSubsystem drive, double desiredDistance, double desiredLateralOffset, boolean isIntake) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        this.desiredLateralOffset = desiredLateralOffset;
        this.isIntake = isIntake;
        addRequirements(drive);

        // PID for forward (distance) control
        distanceController = new PIDController(3.5, 0.0, 0.00);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);

        // PID for lateral offset correction
        lateralController = new PIDController(1.5, 0.0, 0.005);
        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);

        // PID for rotation to face the desired offset position
        rotationController = new PIDController(0.1, 0.0, 0.005);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180); // angle wrap-around
    }

    @Override
    public void initialize() {
        distanceController.reset();
        lateralController.reset();
        rotationController.reset();
        lateralOffsetInitialized = false;
        
        // If in intake mode, select the appropriate camera based on lateral offset
        if (isIntake) {
            // If lateral offset is positive (to the right), use left camera (index 2)
            // If lateral offset is negative (to the left), use right camera (index 1)
            if (desiredLateralOffset > 0) {
                selectedCameraIndex = LEFT_CAMERA_INDEX;
                System.out.println("Intake mode: Using left camera for right-side approach");
            } else {
                selectedCameraIndex = RIGHT_CAMERA_INDEX;
                System.out.println("Intake mode: Using right camera for left-side approach");
            }
        } else {
            selectedCameraIndex = -1; // No specific camera in non-intake mode
        }
        
        System.out.printf("ApproachTagCommand initialized - Target distance: %.2f meters, Lateral offset: %.2f meters, Intake mode: %b%n", 
            desiredDistance, desiredLateralOffset, isIntake);
    }

    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        boolean validTagDetection = false;
        
        // Handle tag detection based on mode
        if (isIntake && selectedCameraIndex != -1) {
            // In intake mode, only use the selected camera
            int detectedTag = poseEstimator.getLastTagDetectedByCamera(selectedCameraIndex);
            double lastDetectionTime = poseEstimator.getLastCameraDetectionTimestamp(selectedCameraIndex);
            double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            
            // Check if we have a recent detection from the selected camera
            if (detectedTag != -1 && (currentTime - lastDetectionTime) < LOST_TAG_TIMEOUT) {
                validTagDetection = true;
                lastTagTimestamp = lastDetectionTime;
            }
        } else {
            // In standard mode, use any camera
            if (poseEstimator.getLastDetectedTagId() != -1) {
                validTagDetection = true;
                lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            }
        }

        if (validTagDetection) {
            double currentDistance = poseEstimator.getDistanceToTag();
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag();
            double currentRotation = poseEstimator.getTagOrientationErrorDeg();

            // Compute forward speed correction (positive drives forward)
            double forwardSpeed = -distanceController.calculate(currentDistance, desiredDistance);
            // Compute lateral correction to maintain the locked lateral offset
            double lateralSpeed = -lateralController.calculate(currentLateralOffset, desiredLateralOffset);

            double rotationSpeed = -rotationController.calculate(currentRotation, 180);

            // Clamp each speed to its maximum limit
            forwardSpeed = Math.min(Math.max(forwardSpeed, -MAX_FORWARD_SPEED), MAX_FORWARD_SPEED);
            lateralSpeed = Math.min(Math.max(lateralSpeed, -MAX_LATERAL_SPEED), MAX_LATERAL_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            // Drive the robot with the computed speeds
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
        
        // Check for valid tag based on mode
        if (isIntake && selectedCameraIndex != -1) {
            int detectedTag = poseEstimator.getLastTagDetectedByCamera(selectedCameraIndex);
            double lastDetectionTime = poseEstimator.getLastCameraDetectionTimestamp(selectedCameraIndex);
            
            if (detectedTag != -1 && (currentTime - lastDetectionTime) < LOST_TAG_TIMEOUT) {
                validTagDetection = true;
                currentDistance = poseEstimator.getDistanceToTag();
                currentLateralOffset = poseEstimator.getLateralOffsetToTag();
                currentRotation = poseEstimator.getTagOrientationErrorDeg();
            }
        } else {
            if (poseEstimator.getLastDetectedTagId() != -1) {
                validTagDetection = true;
                currentDistance = poseEstimator.getDistanceToTag();
                currentLateralOffset = poseEstimator.getLateralOffsetToTag();
                currentRotation = poseEstimator.getTagOrientationErrorDeg();
            }
        }

        if (validTagDetection) {
            // Check distance and lateral offset are within tolerance
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