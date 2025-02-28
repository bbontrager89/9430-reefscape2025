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

    private double lastTagTimestamp = 0;
    // Variables to lock in the initial lateral offset when the tag is first seen
    private boolean lateralOffsetInitialized = false;
    private double desiredLateralOffset = 0;

    public ApproachTagCommand(DriveSubsystem drive, double desiredDistance, double desiredLateralOffset) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        this.desiredLateralOffset = desiredLateralOffset;
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
        System.out.printf("ApproachTagCommand initialized - Target distance: %.2f meters, Lateral offset: %.2f meters%n", 
            desiredDistance, desiredLateralOffset);
    }

    // Helper: Normalize angle to [-180, 180)
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();

        if (poseEstimator.getLastDetectedTagId() != -1) {
            lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();

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
            // No tag detected; stop the robot
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

        if (poseEstimator.getLastDetectedTagId() != -1) {
            double currentDistance = poseEstimator.getDistanceToTag();
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag();
            double currentRotation = poseEstimator.getTagOrientationErrorDeg();

            // Check distance and lateral offset are within tolerance
            boolean distanceOk = Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS;
            boolean lateralOk = Math.abs(currentLateralOffset - desiredLateralOffset) < LATERAL_TOLERANCE_METERS;
            
            double rotationError = currentRotation - 180;
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