package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DoIntakeCoralFromStationCommand extends SequentialCommandGroup {
    private final DriveSubsystem drive;
    private final double desiredLateralOffset;
    private final double desiredDistance;

    @SuppressWarnings("unlikely-arg-type")
    public DoIntakeCoralFromStationCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coralSubsystem,
            DriveSubsystem drive, double desiredLateralOffset) {
        this.drive = drive;
        this.desiredLateralOffset = desiredLateralOffset;
        this.desiredDistance = OIConstants.coralIntakeDistance;

        System.out.printf("ElevatorCommand created - Target lateral offset: %.2f m, Target distance: %.2f m%n",
                desiredLateralOffset, desiredDistance);

        // Only proceed if we initially see a tag
        addRequirements(drive, elevator);

        

        addCommands(
                new ConditionalCommand(
                        // If we see a tag, execute the full alignment sequence
                        new SequentialCommandGroup(
                                Commands.either(new MoveElevator(elevator, 0), new InstantCommand(), () -> hasTag()),
                                Commands.either(new PivotCoral(coralSubsystem, CoralManipulatorConstants.intakePivotPosition), new InstantCommand(), () -> hasTag()),
                                Commands.either(new ApproachTagCommand(drive, OIConstants.coralIntakeDistance, desiredLateralOffset), new InstantCommand(), () -> hasTag()),
                                Commands.either(new IntakeCoral(coralSubsystem, -1, 1.5), new InstantCommand(), () -> hasTag()),
                                Commands.either(new SetCoralSpeed(coralSubsystem, 0), new InstantCommand(), () -> hasTag()),
                                Commands.either(new InstantCommand(() -> {
                                    drive.drive(-0.2, 0, 0, false);
                                }), new InstantCommand(), () -> hasTag()),
                                Commands.either(new WaitCommand(0.25), new InstantCommand(), () -> hasTag()),
                                Commands.either(new InstantCommand(() -> {
                                    drive.drive(0, 0, 0, false);
                                }), new InstantCommand(), () -> hasTag()),
                                new TransitModeCommand(elevator, coralSubsystem)),
                        // If we don't see a tag, do nothing
                        new InstantCommand(),
                        () -> hasTag()));
    }

    private boolean hasTag() {
        int detectedTag = drive.getPoseEstimatorSubsystem().getLastDetectedTagId();
        List<Integer> scoringTagsList = Arrays.stream(AprilTagConstants.intakeStationAprilTags)
                .boxed()
                .toList();
        return scoringTagsList.contains(detectedTag)
                && drive.getPoseEstimatorSubsystem().hasSideCameraDetection();
    }
}