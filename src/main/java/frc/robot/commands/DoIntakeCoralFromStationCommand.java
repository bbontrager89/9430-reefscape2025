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
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.utils.ControllerUtils;

public class DoIntakeCoralFromStationCommand extends SequentialCommandGroup {
        private final DriveSubsystem drive;
        private final double desiredLateralOffset;
        private final double desiredDistance;
        private final double desiredAngle;
        private final PhotonCamera aligningCamera;


        public DoIntakeCoralFromStationCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coralSubsystem,
                        DriveSubsystem drive) {
                this.drive = drive;
                this.aligningCamera = (hasLeftCameraDetection())? VisionConstants.BACK_LEFT_CAMERA 
                                    : VisionConstants.BACK_RIGHT_CAMERA;

                if (aligningCamera != null && aligningCamera.getName().equals(VisionConstants.BL_CAMERA_NAME)) {
                        this.desiredLateralOffset = -0.04;
                        this.desiredDistance = 1.52;
                } else { // Back Right Camera
                        this.desiredLateralOffset = -0.04;
                        this.desiredDistance = 1.52;
                }
                this.desiredAngle = 180;

                addRequirements(drive, elevator, coralSubsystem);

                addCommands(
                        new ConditionalCommand(
                                // If we see a tag, execute the full alignment sequence
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                                this.drive.drive(0, 0, 0, false);
                                        }),
                                        new MoveElevator(elevator, 0),
                                        new PivotCoral(coralSubsystem,
                                                        CoralManipulatorConstants.intakePivotPosition),
                                        Commands.either(
                                                new ApproachStationCommand(this.drive, desiredDistance, desiredLateralOffset, desiredAngle, aligningCamera), 
                                                new ApproachStationCommand(this.drive, desiredDistance, desiredLateralOffset, desiredAngle, aligningCamera).withTimeout(5),
                                                () -> !DriverStation.isAutonomous()),
                                        new IntakeCoral(coralSubsystem, -1, 3),
                                        new SetCoralSpeed(coralSubsystem, 0),
                                        new InstantCommand(() -> {
                                                this.drive.drive(-0.2, 0, 0, false);
                                        }),
                                        new WaitCommand(0.25),
                                        new InstantCommand(() -> {
                                                this.drive.drive(0, 0, 0, false);
                                        }),
                                        new TransitModeCommand(elevator, coralSubsystem)),
                        // If we don't see a tag, do nothing
                        new InstantCommand(() -> {
                                ControllerUtils.Rumble(
                                        RobotContainer.c_driverController.getHID(), 0.2, 1);

                                ControllerUtils.Rumble(
                                        RobotContainer.c_operatorController.getHID(), 0.2, 1);
                        }),
                        () -> aligningCamera != null && hasCameraDetectedTag(aligningCamera)));

        }

        private boolean hasRightCameraDetection() {
                PhotonCamera selectedCameraIndex = VisionConstants.BACK_RIGHT_CAMERA;  // Front camera for left-side approach
                
                int detectedTag = drive.getPoseEstimatorSubsystem().getLastTagDetectedByCamera(selectedCameraIndex);
                List<Integer> scoringTagsList = Arrays.stream(AprilTagConstants.intakeStationAprilTags)
                        .boxed()
                        .toList();
                return scoringTagsList.contains(detectedTag)
                        && drive.getPoseEstimatorSubsystem().hasCameraDetectedTag(selectedCameraIndex);
        }
        
        private boolean hasLeftCameraDetection() {
                PhotonCamera selectedCameraIndex = VisionConstants.BACK_LEFT_CAMERA;  // Front camera for right-side approach
                        
                int detectedTag = drive.getPoseEstimatorSubsystem().getLastTagDetectedByCamera(selectedCameraIndex);
                List<Integer> scoringTagsList = Arrays.stream(AprilTagConstants.intakeStationAprilTags)
                        .boxed()
                        .toList();
                return scoringTagsList.contains(detectedTag)
                        && drive.getPoseEstimatorSubsystem().hasCameraDetectedTag(selectedCameraIndex);
        }

        private boolean hasCameraDetectedTag(PhotonCamera camera) {
                return ((camera.getName().equals(VisionConstants.BR_CAMERA_NAME)) 
                        ? hasRightCameraDetection() 
                        : hasLeftCameraDetection());
        }
}