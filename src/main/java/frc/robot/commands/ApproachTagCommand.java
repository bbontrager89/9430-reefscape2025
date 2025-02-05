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
    private static final double ROTATION_TOLERANCE_DEG = 3.0;
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
        
        // More aggressive PID for approach
        approachController = new PIDController(0.8, 0.1, 0.05);
        approachController.setTolerance(DISTANCE_TOLERANCE_METERS);
        
        // Rotation control during approach
        rotationController = new PIDController(0.08, 0.001, 0.003);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void initialize() {
        approachController.reset();
        rotationController.reset();
        lastApproachSpeed = 0;
        System.out.printf("ApproachTagCommand initialized - Target distance: %.2f meters%n", 
            desiredDistance);
    }
    
    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        
        if (poseEstimator.getLastDetectedTagId() != -1) {
            lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            
            double currentDistance = poseEstimator.getDistanceToTag();
            double approachSpeed = approachController.calculate(currentDistance, desiredDistance);
            
            // Apply acceleration limiting
            double period = 0.02; // 50Hz
            double maxDeltaSpeed = MAX_APPROACH_ACCELERATION * period;
            double deltaSpeed = approachSpeed - lastApproachSpeed;
            deltaSpeed = Math.max(Math.min(deltaSpeed, maxDeltaSpeed), -maxDeltaSpeed);
            approachSpeed = lastApproachSpeed + deltaSpeed;
            lastApproachSpeed = approachSpeed;
            
            double rotationSpeed = rotationController.calculate(poseEstimator.getBearingToTagDeg(), 0);
            
            // Clamp speeds
            approachSpeed = Math.min(Math.max(approachSpeed, -MAX_APPROACH_SPEED), MAX_APPROACH_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            System.out.printf("Approach - Current Distance: %.2f m, Target: %.2f m, Speed: %.2f m/s, Rotation: %.2f rad/s%n",
                currentDistance, desiredDistance, approachSpeed, rotationSpeed);
            
            drive.driveRobotRelative(new ChassisSpeeds(approachSpeed, 0, rotationSpeed));
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
            return Math.abs(currentDistance - desiredDistance) < DISTANCE_TOLERANCE_METERS &&
                   Math.abs(poseEstimator.getBearingToTagDeg()) < ROTATION_TOLERANCE_DEG;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("ApproachTagCommand ended. Interrupted: " + interrupted);
    }
}