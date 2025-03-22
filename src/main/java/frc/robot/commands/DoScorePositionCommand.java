package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.ReefConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.utils.ControllerUtils;

public class DoScorePositionCommand extends SequentialCommandGroup {
    private final DriveSubsystem drive;


    public DoScorePositionCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coralSubsystem,
            DriveSubsystem drive, int scoringPosition, double pivotHeight, PhotonCamera aligningCamera) {
        this.drive = drive;
        
        addRequirements(drive, elevator);


        addCommands(
                new ConditionalCommand(
                        // If we see a tag, execute the full alignment sequence
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        drive.drive(0, 0, 0, false);
                                    }),
                                new PivotCoral(coralSubsystem, pivotHeight),
                                new MoveElevator(elevator, scoringPosition),
                                Commands.either(
                                    new ApproachReefCommand(drive, ReefConstants.REEF_SCORING_DIST, aligningCamera), 
                                    new ApproachReefCommand(drive, ReefConstants.REEF_SCORING_DIST, aligningCamera).withTimeout(2),
                                    () -> !DriverStation.isAutonomous()),
                                new WaitUntilCommand(() -> elevator.atHeight()).withTimeout(1.75),
                                new WaitCommand(0.25),
                                // Eject if tag is seen, else rumble
                                Commands.either(
                                    new SequentialCommandGroup(
                                        new SetCoralSpeed(coralSubsystem, (scoringPosition == 1)? 0.4 : 1),
                                        new WaitCommand(0.7),
                                        new SetCoralSpeed(coralSubsystem, 0)),

                                    new InstantCommand(() -> {
                                        ControllerUtils.Rumble(
                                                RobotContainer.c_driverController.getHID(), 0.2, 1);
            
                                        ControllerUtils.Rumble(
                                                RobotContainer.c_operatorController.getHID(), 0.2, 1);
                                        }), 

                                    () -> (hasCameraDetectedTag(aligningCamera) && elevator.atHeight()) || DriverStation.isAutonomous()),

                                new TransitModeCommand(elevator, coralSubsystem)),
                        // If we don't see a tag, do nothing
                        new InstantCommand(() -> {
                            ControllerUtils.Rumble(
                                    RobotContainer.c_driverController.getHID(), 0.2, 1);

                            ControllerUtils.Rumble(
                                    RobotContainer.c_operatorController.getHID(), 0.2, 1);
                            }),
                        () -> hasCameraDetectedTag(aligningCamera)));


    }

    private boolean hasRightCameraDetection() {
        PhotonCamera selectedCameraIndex = VisionConstants.FRONT_RIGHT_CAMERA;  // Front camera for right-side approach
            
        int detectedTag = drive.getPoseEstimatorSubsystem().getLastTagDetectedByCamera(selectedCameraIndex);
        List<Integer> scoringTagsList = Arrays.stream(AprilTagConstants.scoringAprilTags)
                .boxed()
                .toList();
        return scoringTagsList.contains(detectedTag)
                && drive.getPoseEstimatorSubsystem().hasCameraDetectedTag(selectedCameraIndex);
    }

    private boolean hasLeftCameraDetection() {
        PhotonCamera selectedCameraIndex = VisionConstants.FRONT_LEFT_CAMERA;  // Front camera for right-side approach
            
        int detectedTag = drive.getPoseEstimatorSubsystem().getLastTagDetectedByCamera(selectedCameraIndex);
        List<Integer> scoringTagsList = Arrays.stream(AprilTagConstants.scoringAprilTags)
                .boxed()
                .toList();
        return scoringTagsList.contains(detectedTag)
                && drive.getPoseEstimatorSubsystem().hasCameraDetectedTag(selectedCameraIndex);
    }

    private boolean hasCameraDetectedTag(PhotonCamera camera) {
        return ((camera.getName().equals(VisionConstants.FR_CAMERA_NAME)) 
                ? hasRightCameraDetection() 
                : hasLeftCameraDetection());
    }
    
}