package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ApproachTagCommand extends Command {
    private final DriveSubsystem drive;
    private final double desiredDistance;
    private final PIDController approachController;
    private final PIDController rotationController;
    
    // Constants
    private static final double DISTANCE_TOLERANCE_METERS = 0.08;  // 8cm
    private static final double ROTATION_TOLERANCE_DEG = 2.0;     // Increased for stability
    private static final double MAX_APPROACH_SPEED = 1.5; // m/s
    private static final double MAX_ROTATION_SPEED = 1.0; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds
    
    private double lastTagTimestamp = 0;
    private double lastApproachSpeed = 0;
    private static final double MAX_APPROACH_ACCELERATION = 2.0; // m/s^2
    
    public ApproachTagCommand(DriveSubsystem drive, double desiredDistance) {
        this.drive = drive;
        this.desiredDistance = desiredDistance;
        addRequirements(drive);
        
        // PID for approach distance control
        approachController = new PIDController(2.0, 0.1, 0.05);
        approachController.setTolerance(DISTANCE_TOLERANCE_METERS);
        
        // PID for maintaining perpendicular alignment
        rotationController = new PIDController(0.08, 0.001, 0.003);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180);  // This is key for handling wrap-around
    }
    
    @Override
    public void initialize() {
        approachController.reset();
        rotationController.reset();
        lastApproachSpeed = 0;
        System.out.printf("ApproachTagCommand initialized - Target distance: %.2f meters%n", 
            desiredDistance);
    }
    
    private double normalizeAngle(double angle) {
        // Normalize angle to [-180, 180)
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
    
    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        
        if (poseEstimator.getLastDetectedTagId() != -1) {
            lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            
            // Get current state
            double currentDistance = poseEstimator.getDistanceToTag();
            double tagOrientation = normalizeAngle(poseEstimator.getTagOrientationErrorDeg());
            
            // Calculate approach speed
            double approachSpeed = -approachController.calculate(currentDistance, desiredDistance);
            
            // Calculate rotation to maintain perpendicular alignment
            double rotationSpeed = -rotationController.calculate(tagOrientation, 0);
            
            // Clamp speeds
            approachSpeed = Math.min(Math.max(approachSpeed, -MAX_APPROACH_SPEED), MAX_APPROACH_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            // Reduce rotation speed when close to target
            double distanceRatio = (currentDistance - desiredDistance) / desiredDistance;
            rotationSpeed *= Math.min(1.0, Math.max(0.2, distanceRatio));
            
            // Reduce approach speed when alignment is poor
            double alignmentFactor = Math.cos(Math.toRadians(tagOrientation));
            approachSpeed *= Math.max(0.2, alignmentFactor); // Minimum 20% speed
            
            // Additional slow down when very close to target
            if (Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS * 2) {
                approachSpeed *= 0.5;
                rotationSpeed *= 0.5;
            }
            
            System.out.printf("Approach - Distance: %.2f m, Target: %.2f m, Speed: %.2f m/s, " +
                            "Orientation: %.2f°, Rotation: %.2f rad/s%n",
                currentDistance, desiredDistance, approachSpeed, tagOrientation, rotationSpeed);
            
            // Apply motion
            drive.driveRobotRelative(new ChassisSpeeds(approachSpeed, 0, 0));
        } else {
            drive.driveRobotRelative(new ChassisSpeeds());
            lastApproachSpeed = 0;
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
            double tagOrientation = normalizeAngle(poseEstimator.getTagOrientationErrorDeg());
            
            // Only finish when both distance and orientation are within tolerance
            return Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS &&
                   Math.abs(tagOrientation) < ROTATION_TOLERANCE_DEG;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("ApproachTagCommand ended. Interrupted: " + interrupted);
    }
}