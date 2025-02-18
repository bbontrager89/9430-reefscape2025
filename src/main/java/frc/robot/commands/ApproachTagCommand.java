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
    private static final double DISTANCE_TOLERANCE_METERS = 0.03; // 3cm tolerance
    private static final double LATERAL_TOLERANCE_METERS = 0.03;  // 3cm tolerance
    private static final double ROTATION_TOLERANCE_DEG = 2.0;       // degrees tolerance for rotation error
    private static final double MAX_FORWARD_SPEED = 1.5;            // m/s
    private static final double MAX_LATERAL_SPEED = 1.0;            // m/s
    private static final double MAX_ROTATION_SPEED = 0.3;           // rad/s (small correction)
    private static final double LOST_TAG_TIMEOUT = 0.7;             // seconds

    private double lastTagTimestamp = 0;
    // Lock-in flag for the lateral offset (set by constructor) and heading
    private boolean lateralOffsetInitialized = false;
    private boolean headingLocked = false;
    // The lateral offset is now provided via the constructor.
    // The lockedAngle is computed once when the tag is first seen.
    private double lockedAngle = 0;

    public ApproachTagCommand(DriveSubsystem drive, double desiredDistance, double desiredLateralOffset) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        // Here, desiredLateralOffset is assumed to be the target lateral offset.
        // (It could be zero if you want the robot to center on the tag, or a positive/negative value if you want an offset.)
        addRequirements(drive);

        // PID for forward (distance) control.
        distanceController = new PIDController(2.0, 0.0, 0.00);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);

        // PID for lateral offset correction.
        lateralController = new PIDController(0.5, 0.0, 0.0);
        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);

        // PID for slight rotation correction.
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

            // When the tag is first seen, lock in the lateral offset and the desired (locked) heading.
            if (!lateralOffsetInitialized) {
                lateralOffsetInitialized = true;
                // Calculate the offset angle to the tag based on its current measured position.
                // This angle is relative to the robot's forward axis.
                double measuredOffsetAngle = Math.toDegrees(Math.atan2(currentLateralOffset, currentDistance));
                // Lock in the desired heading by adding the measured offset angle to the current heading.
                lockedAngle = drive.getHeading() + measuredOffsetAngle;
                System.out.printf("Locked lateral offset. Desired heading locked at: %.2f° (measured offset: %.2f°)%n",
                        lockedAngle, measuredOffsetAngle);
            }

            // Compute forward and lateral speed corrections using PID controllers.
            double forwardSpeed = -distanceController.calculate(currentDistance, desiredDistance);
            double lateralSpeed = -lateralController.calculate(currentLateralOffset, /* target offset */ 0);
            // (If you want the lateral target to be nonzero, replace the zero above with the desired lateral offset.)

            // Apply rotation correction only until the robot's heading reaches the locked heading.
            double rotationSpeed = 0;
            if (!headingLocked) {
                double currentHeading = drive.getHeading();
                double rotationError = normalizeAngle(lockedAngle - currentHeading);
                rotationSpeed = rotationController.calculate(rotationError, 0);
                if (Math.abs(rotationError) < ROTATION_TOLERANCE_DEG) {
                    headingLocked = true;
                    rotationSpeed = 0;
                    System.out.printf("Heading locked at: %.2f°%n", currentHeading);
                }
            }

            // Clamp each speed to its maximum limit.
            forwardSpeed = Math.min(Math.max(forwardSpeed, -MAX_FORWARD_SPEED), MAX_FORWARD_SPEED);
            lateralSpeed = Math.min(Math.max(lateralSpeed, -MAX_LATERAL_SPEED), MAX_LATERAL_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);

            System.out.printf("Approach - Dist: %.2f m (Target: %.2f m), Lateral measured: %.2f m, " +
                            "Gyro: %.2f°, Locked Heading: %.2f°, Error: %.2f°, " +
                            "Forward: %.2f m/s, Lateral: %.2f m/s, Rot: %.2f rad/s%n",
                    currentDistance, desiredDistance, currentLateralOffset,
                    drive.getHeading(), lockedAngle, normalizeAngle(lockedAngle - drive.getHeading()),
                    forwardSpeed, lateralSpeed, rotationSpeed);

            // Drive the robot with the computed speeds.
            drive.driveRobotRelative(new ChassisSpeeds(forwardSpeed, lateralSpeed, rotationSpeed));
        } else {
            // If no tag is detected, stop the robot.
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
            // We finish when the robot is within the distance tolerance.
            // (Rotation adjustment is only applied initially, so we don't check it here.)
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
