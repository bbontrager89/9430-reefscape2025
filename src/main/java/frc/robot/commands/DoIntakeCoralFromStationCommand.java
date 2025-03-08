package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.utils.ControllerUtils;

public class DoIntakeCoralFromStationCommand extends SequentialCommandGroup {
        private final DriveSubsystem drive;
        private final double desiredLateralOffset;
        private final double desiredDistance;

        @SuppressWarnings("unlikely-arg-type")
        public DoIntakeCoralFromStationCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coralSubsystem,
                        DriveSubsystem drive) {
                this.drive = drive;
                this.desiredLateralOffset = (drive.getPoseEstimatorSubsystem().getLastDetectionCameraIndex() == 1)
                                ? OIConstants.intakePositionLeft
                                : OIConstants.intakePositionRight;
                this.desiredDistance = (drive.getPoseEstimatorSubsystem().getLastDetectionCameraIndex() == 1)
                                ? OIConstants.rightCoralIntakeDistance
                                : OIConstants.leftCoralIntakeDistance;

                System.out.printf("ElevatorCommand created - Target lateral offset: %.2f m, Target distance: %.2f m%n",
                                desiredLateralOffset, desiredDistance);

                addRequirements(drive, elevator);

                addCommands(
                        new ConditionalCommand(
                                // If we see a tag, execute the full alignment sequence
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> {
                                                this.drive.drive(0, 0, 0, false);
                                        }),
                                        new StrafeToAlignCommand(drive, desiredLateralOffset, true).withTimeout(1.5),
                                        new MoveElevator(elevator, 0),
                                        new PivotCoral(coralSubsystem,
                                                        CoralManipulatorConstants.intakePivotPosition),
                                        Commands.either(
                                                new ApproachTagCommand(this.drive, desiredDistance, desiredLateralOffset, true), 
                                                new ApproachTagCommand(this.drive, desiredDistance, desiredLateralOffset, true).withTimeout(3),
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
                        () -> hasTag()));

        }

        private boolean hasTag() {
                int detectedTagRight = drive.getPoseEstimatorSubsystem().getLastTagDetectedByCamera(1);
                int detectedTagLeft = drive.getPoseEstimatorSubsystem().getLastTagDetectedByCamera(2);
                List<Integer> scoringTagsList = Arrays.stream(AprilTagConstants.intakeStationAprilTags)
                                .boxed()
                                .toList();
                return (scoringTagsList.contains(detectedTagLeft) || scoringTagsList.contains(detectedTagRight))
                                && drive.getPoseEstimatorSubsystem().hasSideCameraDetection();
        }
}