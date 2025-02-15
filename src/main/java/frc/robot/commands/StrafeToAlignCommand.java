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
    private static final double LATERAL_TOLERANCE_METERS = 0.07;  // meters
    private static final double ROTATION_TOLERANCE_DEG = 1.7;
    private static final double MAX_STRAFE_SPEED = 1.0; // m/s
    private static final double MAX_ROTATION_SPEED = 0.5; // rad/s
    private static final double LOST_TAG_TIMEOUT = 0.5; // seconds
    
    // --- Dynamic P gain constants for strafe controller ---
    private static final double LOW_P_GAIN = 1.0;    // lower P gain when close to target
    private static final double FULL_P_GAIN = 3.8;   // full P gain (as originally used)
    private static final double P_RAMP_THRESHOLD = 0.2; // error threshold (meters) to reach full P gain

    private double lastTagTimestamp = 0;
    private double lastStrafeSpeed = 0;
    private static final double MAX_STRAFE_ACCELERATION = 1.0; // m/s^2
    
    public StrafeToAlignCommand(DriveSubsystem drive, double desiredLateralOffset) {
        this.drive = drive;
        this.desiredLateralOffset = desiredLateralOffset;
        addRequirements(drive);
        
        // PID for strafe control.
        // We initially set the P gain to FULL_P_GAIN but will adjust it dynamically.
        strafeController = new PIDController(FULL_P_GAIN, 0.0, 0.0);
        strafeController.setTolerance(LATERAL_TOLERANCE_METERS);
        
        // PID for rotation.
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
            
            // Compute the lateral error
            double lateralError = currentLateralOffset - desiredLateralOffset;
            double absError = Math.abs(lateralError);
            
            // Dynamically adjust the P gain: if the error is small, use a lower P gain.
            // Once the error exceeds P_RAMP_THRESHOLD, we use FULL_P_GAIN.
            double effectivePGain = LOW_P_GAIN 
                    + (FULL_P_GAIN - LOW_P_GAIN) * Math.min(absError / P_RAMP_THRESHOLD, 1.0);
            strafeController.setP(effectivePGain);
            
            // Calculate desired speeds using the dynamically adjusted P gain.
            // Note: strafeSpeed is negated because a positive lateral offset indicates that
            // the robot needs to move left (i.e. negative Y direction).
            double strafeSpeed = -strafeController.calculate(currentLateralOffset, desiredLateralOffset);
            double rotationSpeed = rotationController.calculate(currentOrientation, 0);
            
            // (Optional) Apply acceleration limiting to the strafe speed if needed.
           /* double period = 0.04; // Assuming 50Hz update rate
            double maxDeltaSpeed = MAX_STRAFE_ACCELERATION * period;
            double deltaSpeed = strafeSpeed - lastStrafeSpeed;
            deltaSpeed = Math.max(Math.min(deltaSpeed, maxDeltaSpeed), -maxDeltaSpeed);
            strafeSpeed = lastStrafeSpeed + deltaSpeed;
            lastStrafeSpeed = strafeSpeed;*/
            
            // Clamp speeds
            strafeSpeed = Math.min(Math.max(strafeSpeed, -MAX_STRAFE_SPEED), MAX_STRAFE_SPEED);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            
            // Log the current state including the effective P gain for debugging.
            System.out.printf("Strafe - Lateral Error: %.2f m, Effective P Gain: %.2f, Speed: %.2f m/s, Rotation: %.2f rad/s%n",
                lateralError, effectivePGain, strafeSpeed, rotationSpeed);
            
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
        
        // Check if we're aligned and stable
        if (poseEstimator.getLastDetectedTagId() != -1) {
            double currentLateralOffset = poseEstimator.getLateralOffsetToTag();
            double orientationError = poseEstimator.getTagOrientationErrorDeg();
            
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
