package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ApproachTagCommand extends Command {
    private final DriveSubsystem drive;
    private final double desiredDistance;
    private final double desiredLateralOffset;
    
    private final PIDController approachController;
    private final PIDController lateralController;
    private final PIDController rotationController;
    
    // Tolerances and maximum speeds/constants
    private static final double DISTANCE_TOLERANCE_METERS = 0.08;  // 8 cm
    private static final double LATERAL_TOLERANCE_METERS = 0.07;   // 7 cm
    private static final double ROTATION_TOLERANCE_DEG = 2.0;      // 2 degrees

    private static final double MAX_APPROACH_SPEED = 1.5; // m/s
    private static final double MAX_STRAFE_SPEED = 1.0;   // m/s
    private static final double MAX_ROTATION_SPEED = 1.0;   // rad/s

    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds
    
    private double lastTagTimestamp = 0;
    
    public ApproachTagCommand(DriveSubsystem drive, double desiredDistance, double desiredLateralOffset) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        this.desiredLateralOffset = desiredLateralOffset;
        addRequirements(drive);
        
        // PID for forward (approach) distance control
        approachController = new PIDController(2.0, 0.1, 0.05);
        approachController.setTolerance(DISTANCE_TOLERANCE_METERS);
        
        // PID for lateral (strafe) offset control
        lateralController = new PIDController(3.8, 0.0, 0.0);
        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);
        
        // PID for maintaining perpendicular alignment
        rotationController = new PIDController(0.08, 0.001, 0.003);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void initialize() {
        approachController.reset();
        lateralController.reset();
        rotationController.reset();
        lastTagTimestamp = Timer.getFPGATimestamp();
        System.out.printf("ApproachTagWithLateralOffsetCommand initialized - Target distance: %.2f m, Lateral offset: %.2f m%n",
                desiredDistance, desiredLateralOffset);
    }
    
    // Normalize an angle to the range [-180, 180)
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        }
        if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }
    
    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        
        // Check if the tag is currently detected
        if (poseEstimator.getLastDetectedTagId() != -1) {
            // Update timestamp whenever we see the tag
            lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            
            // Retrieve sensor data from the pose estimator
            double currentDistance = poseEstimator.getDistanceToTag();
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag();
            double tagOrientation = normalizeAngle(poseEstimator.getTagOrientationErrorDeg());
            
            // Calculate corrections using PID controllers
            double forwardSpeed = -approachController.calculate(currentDistance, desiredDistance);
            double strafeSpeed = -lateralController.calculate(currentLateralOffset, desiredLateralOffset);
            double rotationSpeed = -rotationController.calculate(tagOrientation, 0);
            
            // Optionally scale down rotation when close to the target
            double distanceRatio = (currentDistance - desiredDistance) / desiredDistance;
            rotationSpeed *= Math.min(1.0, Math.max(0.2, distanceRatio));
            
            // Apply additional slowdown when nearly at the target for finer control
            if (Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS * 2) {
                forwardSpeed *= 0.5;
                strafeSpeed *= 0.5;
                rotationSpeed *= 0.5;
            }
            
            // Clamp speeds to ensure they do not exceed maximum allowed speeds
            forwardSpeed = clamp(forwardSpeed, -MAX_APPROACH_SPEED, MAX_APPROACH_SPEED);
            strafeSpeed = clamp(strafeSpeed, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
            rotationSpeed = clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
            
            System.out.printf("ApproachTagWithLateralOffset - Distance: %.2f m (Target: %.2f m), Lateral: %.2f m (Target: %.2f m), " +
                              "Forward: %.2f m/s, Strafe: %.2f m/s, Orientation: %.2f°, Rotation: %.2f rad/s%n",
                              currentDistance, desiredDistance, currentLateralOffset, desiredLateralOffset,
                              forwardSpeed, strafeSpeed, tagOrientation, rotationSpeed);
            
            // Command the drive subsystem with combined corrections
            drive.driveRobotRelative(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));
        } else {
            // If the tag is lost (for example, when too close), stop the robot
            drive.driveRobotRelative(new ChassisSpeeds());
        }
    }
    
    @Override
    public boolean isFinished() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        double currentTime = Timer.getFPGATimestamp();
        
        // If the tag has not been seen for a set timeout, finish the command.
        // This is important because the vision system may lose the tag when the robot is too close.
        if (currentTime - lastTagTimestamp > LOST_TAG_TIMEOUT) {
            System.out.println("Tag lost - finishing command.");
            return true;
        }
        
        // If the tag is detected, finish when the error is within tolerances
        if (poseEstimator.getLastDetectedTagId() != -1) {
            double currentDistance = poseEstimator.getDistanceToTag();
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag();
            double tagOrientation = normalizeAngle(poseEstimator.getTagOrientationErrorDeg());
            
            return Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS &&
                   Math.abs(currentLateralOffset - desiredLateralOffset) < LATERAL_TOLERANCE_METERS &&
                   Math.abs(tagOrientation) < ROTATION_TOLERANCE_DEG;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("ApproachTagWithLateralOffsetCommand ended. Interrupted: " + interrupted);
    }
    
    // Utility method to clamp a value between a minimum and maximum
    private double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
