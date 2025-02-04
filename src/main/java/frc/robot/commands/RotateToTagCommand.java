package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToTagCommand extends Command {
    private final DriveSubsystem drive;
    private final PIDController rotationController;
    
    // Constants
    private static final double ROTATION_TOLERANCE_DEG = 2.0;
    private static final double MAX_ROTATION_SPEED = 0.5; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds
    
    // Tracking when we lose sight of the tag
    private double lastTagTimestamp = 0;
    
    public RotateToTagCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
        
        // PID for rotation control
        rotationController = new PIDController(0.05, 0.0, 0.001);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void initialize() {
        rotationController.reset();
        System.out.println("RotateToTagCommand initialized");
    }
    
    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        double bearing = poseEstimator.getBearingToTagDeg();
        
        if (poseEstimator.getLastDetectedTagId() != -1) {
            lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            
            // Calculate rotation speed based on bearing
            double rotationSpeed = -rotationController.calculate(bearing, 0);
            
            // Clamp rotation speed
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            // Log the current state
            System.out.printf("Rotation - Bearing: %.2f°, RotationSpeed: %.2f rad/s%n", 
                bearing, rotationSpeed);
            
            // Apply rotation
            drive.driveRobotRelative(new ChassisSpeeds(0, 0, rotationSpeed));
        } else {
            // If we've lost the tag, stop
            drive.driveRobotRelative(new ChassisSpeeds());
        }
    }
    
    @Override
    public boolean isFinished() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        
        // Get current timestamp
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        // Check if we've lost sight of the tag for too long
        if (currentTime - lastTagTimestamp > LOST_TAG_TIMEOUT) {
            return true;
        }
        
        // Finish if we're aligned within tolerance
        return poseEstimator.getLastDetectedTagId() != -1 && 
               Math.abs(poseEstimator.getBearingToTagDeg()) < ROTATION_TOLERANCE_DEG;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("RotateToTagCommand ended. Interrupted: " + interrupted);
    }
}