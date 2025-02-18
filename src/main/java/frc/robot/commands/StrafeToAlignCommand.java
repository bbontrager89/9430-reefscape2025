package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class StrafeToAlignCommand extends Command {
    private final DriveSubsystem drive;
    private final double desiredLateralOffset;
    private final PIDController strafeController;
    private final PIDController rotationController;
    
    // Constants
    private static final double LATERAL_TOLERANCE_METERS = 0.05;  // 5cm
    private static final double ROTATION_TOLERANCE_DEG = 1.7;
    private static final double MAX_STRAFE_SPEED = 1.0; // m/s
    private static final double MAX_ROTATION_SPEED = 0.5; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds
    
    private double lastTagTimestamp = 0;
    private double lastStrafeSpeed = 0;
    private static final double MAX_STRAFE_ACCELERATION = 1.0; // m/s^2
    
    public StrafeToAlignCommand(DriveSubsystem drive, double desiredLateralOffset) {
        this.drive = drive;
        this.desiredLateralOffset = desiredLateralOffset;
        addRequirements(drive);
        
        // PID for strafe control
        strafeController = new PIDController(3.0, 0.0, 0.0);
        strafeController.setTolerance(LATERAL_TOLERANCE_METERS);
        // PID for rotation
        rotationController = new PIDController(0.3, 0.0, 0.0);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
        rotationController.enableContinuousInput(-180, 180);
    }
    
    @Override
    public void initialize() {
        strafeController.reset();
        rotationController.reset();
        lastStrafeSpeed = 0;
        System.out.printf("StrafeToAlignCommand initialized - Target lateral offset: %.2f meters%n", 
            desiredLateralOffset);
    }
    
    @Override
    public void execute() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        
        if (poseEstimator.getLastDetectedTagId() != -1) {
            lastTagTimestamp = poseEstimator.getLastDetectionTimestamp();
            
            // Get current errors
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag();
            double currentOrientation = poseEstimator.getTagOrientationErrorDeg();
            
            // Calculate desired speeds
            // Note: strafeSpeed is negated because positive lateral offset means robot needs to move left (negative Y)
            double strafeSpeed = -strafeController.calculate(currentLateralOffset, desiredLateralOffset);
            double rotationSpeed = rotationController.calculate(currentOrientation, 0);
            
            // Apply acceleration limiting to strafe
           /* double period = 0.04; // Assuming 50Hz
            double maxDeltaSpeed = MAX_STRAFE_ACCELERATION * period;
            double deltaSpeed = strafeSpeed - lastStrafeSpeed;
            deltaSpeed = Math.max(Math.min(deltaSpeed, maxDeltaSpeed), -maxDeltaSpeed);
            strafeSpeed = lastStrafeSpeed + deltaSpeed;
            lastStrafeSpeed = strafeSpeed;*/
            
            // Clamp speeds
            strafeSpeed = Math.min(Math.max(strafeSpeed, -MAX_STRAFE_SPEED), MAX_STRAFE_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            // Log current state
            System.out.printf("Strafe - Current Lateral: %.2f m, Target: %.2f m, Speed: %.2f m/s, Rotation: %.2f rad/s%n",
                currentLateralOffset, desiredLateralOffset, strafeSpeed, rotationSpeed);
            
            // Apply motion
            drive.driveRobotRelative(new ChassisSpeeds(0, strafeSpeed, rotationSpeed));
        } else {
            // If we've lost the tag, stop
            drive.driveRobotRelative(new ChassisSpeeds());
            lastStrafeSpeed = 0;
        }
    }
    
    @Override
    public boolean isFinished() {
        var poseEstimator = drive.getPoseEstimatorSubsystem();
        
        // Check for timeout
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (currentTime - lastTagTimestamp > LOST_TAG_TIMEOUT) {
            return true;
        }
        
        // Check if we're aligned
        if (poseEstimator.getLastDetectedTagId() != -1) {
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag();
            double orientationError = poseEstimator.getTagOrientationErrorDeg();
            
            // Only finish if we're stable
            return Math.abs(currentLateralOffset - desiredLateralOffset) < LATERAL_TOLERANCE_METERS &&
                   Math.abs(orientationError) < ROTATION_TOLERANCE_DEG &&
                   Math.abs(lastStrafeSpeed) < 0.05;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.driveRobotRelative(new ChassisSpeeds());
        System.out.println("StrafeToAlignCommand ended. Interrupted: " + interrupted);
    }
}