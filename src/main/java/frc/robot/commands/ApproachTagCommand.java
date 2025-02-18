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
    private static final double DISTANCE_TOLERANCE_METERS = 0.03; // meters
    private static final double LATERAL_TOLERANCE_METERS = 0.03;    // meters
    private static final double ROTATION_TOLERANCE_DEG = 2.0;         // degrees
    private static final double MAX_FORWARD_SPEED = 1.5;              // m/s
    private static final double MAX_LATERAL_SPEED = 1.0;              // m/s
    private static final double MAX_ROTATION_SPEED = 0.3;             // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.90;              // seconds

    private double lastTagTimestamp = 0;
    // Variables to lock in the initial lateral offset and heading
    private boolean lateralOffsetInitialized = false;
    private double desiredLateralOffset = 0;
    private boolean headingLocked = false;
    private double desiredHeading = 0; // in degrees

    public ApproachTagCommand(DriveSubsystem drive, double desiredDistance) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        addRequirements(drive);

        // PID for forward (distance) control.
        distanceController = new PIDController(2.0, 0.0, 0.00);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);

        // PID for lateral offset correction.
        lateralController = new PIDController(0.5, 0.0, 0.0);
        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);

        // PID for rotation correction.
        rotationController = new PIDController(0.05, 0.0, 0.005);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180); // angle wrap-around
    }

    @Override
    public void initialize() {
        distanceController.reset();
        lateralController.reset();
        rotationController.reset();
        lateralOffsetInitialized = false;
        headingLocked = false;
        System.out.printf("ApproachTagCommand initialized - Target distance: %.2f meters%n", desiredDistance);
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

            // When the tag is first seen, lock in the lateral offset and compute the target heading.
            if (!lateralOffsetInitialized) {
                desiredLateralOffset = currentLateralOffset;
                // Calculate the offset angle relative to the robot's current forward direction.
                double lockedAngle = Math.toDegrees(Math.atan2(currentLateralOffset, currentDistance));
                // The target (desired) heading is the robot's current heading plus this offset.
                desiredHeading = drive.getGyroAngle() + lockedAngle;
                lateralOffsetInitialized = true;
                System.out.printf("Locked lateral offset: %.2f m, Locked angle: %.2f°, Desired heading: %.2f°%n",
                        desiredLateralOffset, lockedAngle, desiredHeading);
            }

            // Compute forward and lateral speeds based on PID controllers.
            double forwardSpeed = -distanceController.calculate(currentDistance, desiredDistance);
            double lateralSpeed = -lateralController.calculate(currentLateralOffset, desiredLateralOffset);

            double rotationSpeed = 0;
            // Only adjust rotation initially until the robot's heading reaches the desired heading.
            if (!headingLocked) {
                double currentHeading = drive.getGyroAngle();
                double rotationError = normalizeAngle(desiredHeading - currentHeading);
                rotationSpeed = rotationController.calculate(rotationError, 0);
                if (Math.abs(rotationError) < ROTATION_TOLERANCE_DEG) {
                    headingLocked = true;
                    rotationSpeed = 0;
                    System.out.printf("Heading locked at %.2f°%n", currentHeading);
                }
            }

            // Clamp speeds to their maximum values.
            forwardSpeed = Math.min(Math.max(forwardSpeed, -MAX_FORWARD_SPEED), MAX_FORWARD_SPEED);
            lateralSpeed = Math.min(Math.max(lateralSpeed, -MAX_LATERAL_SPEED), MAX_LATERAL_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);

            System.out.printf("Approach - Dist: %.2f m (Target: %.2f m), Lateral: %.2f m (Locked: %.2f m), " +
                            "Current Heading: %.2f°, Desired Heading: %.2f°, " +
                            "Forward: %.2f m/s, Lateral: %.2f m/s, Rot: %.2f rad/s%n",
                    currentDistance, desiredDistance,
                    currentLateralOffset, desiredLateralOffset,
                    drive.getGyroAngle(), desiredHeading,
                    forwardSpeed, lateralSpeed, rotationSpeed);

            drive.driveRobotRelative(new ChassisSpeeds(forwardSpeed, lateralSpeed, rotationSpeed));
        } else {
            drive.driveRobotRelative(new ChassisSpeeds());
        }
    }

    @Override
    public boolean isFinished() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        if (currentTime - lastTagTimestamp > LOST_TAG_TIMEOUT) {
            return true;
        }

        if (poseEstimator.getLastDetectedTagId() != -1) {
            double currentDistance = poseEstimator.getDistanceToTag();
            // Do not finish until heading is locked.
            if (!headingLocked) {
                return false;
            }
            return Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("ApproachTagCommand ended. Interrupted: " + interrupted);
    }
}
