package frc.robot.commands;

import java.io.Console;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.utils.ControllerUtils;

public class DoScorePositionCommand extends SequentialCommandGroup {
    private final DriveSubsystem drive;
    private final double desiredLateralOffset;
    private final double desiredDistance;

    @SuppressWarnings("unlikely-arg-type")
    public DoScorePositionCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coralSubsystem,
            DriveSubsystem drive, int scoringPosition, double desiredLateralOffset, double desiredDistance,
            double pivotHeight) {
        this.drive = drive;
        this.desiredLateralOffset = desiredLateralOffset;
        this.desiredDistance = desiredDistance;

        System.out.printf("ElevatorCommand created - Target lateral offset: %.2f m, Target distance: %.2f m%n",
                desiredLateralOffset, desiredDistance);

        
        addRequirements(drive, elevator);

        addCommands(
                new ConditionalCommand(
                        // If we see a tag, execute the full alignment sequence
                        new SequentialCommandGroup(
                                // new RotateToTagCommand(drive),
                                new InstantCommand(() -> {
                                        drive.drive(0, 0, 0, false);
                                    }),
                                new PivotCoral(coralSubsystem, pivotHeight),
                                new MoveElevator(elevator, scoringPosition),
                                // new StrafeToAlignCommand(drive, desiredLateralOffset),
                                Commands.either(
                                    new ApproachTagCommand(drive, desiredDistance, desiredLateralOffset, false), 
                                    new ApproachTagCommand(drive, desiredDistance, desiredLateralOffset, false).withTimeout(2),
                                    () -> !DriverStation.isAutonomous()),
                                new WaitUntilCommand(() -> elevator.atHeight()).withTimeout(1.75),
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

                                    () -> (hasTag() && elevator.atHeight())),

                                new TransitModeCommand(elevator, coralSubsystem)),
                        // If we don't see a tag, do nothing
                        new InstantCommand(() -> {
                            ControllerUtils.Rumble(
                                    RobotContainer.c_driverController.getHID(), 0.2, 1);

                            ControllerUtils.Rumble(
                                    RobotContainer.c_operatorController.getHID(), 0.2, 1);
                            }),
                        () -> hasTag()));


    }

    private boolean hasTag() {
        int selectedCameraIndex = -1;
        if (desiredLateralOffset > 0) {
            selectedCameraIndex = 0;  // Front camera for right-side approach
            System.out.println("Non-intake mode: Using front camera index 0 for right-side approach");
        } else if (desiredLateralOffset < 0) {
            selectedCameraIndex = 3;  // Front camera for left-side approach
            System.out.println("Non-intake mode: Using front camera index 3 for left-side approach");
        } else {
            selectedCameraIndex = -1; // No specific camera selected; use any available detection
            System.out.println("Non-intake mode: No lateral offset specified, using any available camera");
        }
        if(selectedCameraIndex == -1)return false;
        int detectedTag = drive.getPoseEstimatorSubsystem().getLastTagDetectedByCamera(selectedCameraIndex);
        List<Integer> scoringTagsList = Arrays.stream(AprilTagConstants.scoringAprilTags)
                .boxed()
                .toList();
        return scoringTagsList.contains(detectedTag)
                && drive.getPoseEstimatorSubsystem().hasCameraDetectedTag(selectedCameraIndex);
    }
    
}