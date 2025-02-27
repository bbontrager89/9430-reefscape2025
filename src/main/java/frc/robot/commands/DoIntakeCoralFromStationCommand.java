package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

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

        
        addRequirements(drive, elevator);

        // Only proceed if we initially see a tag
        if (drive.getPoseEstimatorSubsystem().getLastDetectedTagId() != -1) {
            addCommands(
                new ConditionalCommand(
                    // If we see a tag, execute the full alignment sequence
                    new SequentialCommandGroup(
                        new RotateToTagCommand(drive),
                        new StrafeToAlignCommand(drive, desiredLateralOffset),
                        new MoveElevator(elevator, 0),
                        new PivotCoral(coralSubsystem, CoralManipulatorConstants.intakePivotPosition),
                        new ApproachTagCommand(drive, OIConstants.coralIntakeDistance, desiredLateralOffset),
                        new IntakeCoral(coralSubsystem, -1, 1.5),
                        new SetCoralSpeed(coralSubsystem, 0),
                        new InstantCommand(() -> {
                            drive.drive(-0.2, 0, 0, false);
                        }),
                        new WaitCommand(0.25),
                        new InstantCommand(() -> {
                            drive.drive(0, 0, 0, false);
                        }),
                        new TransitModeCommand(elevator, coralSubsystem)
                    ),
                    // If we don't see a tag, do nothing
                    new InstantCommand(),
                    () -> drive.getPoseEstimatorSubsystem().getLastDetectedTagId() != -1
                )
            );
        } else {
            addCommands();
        }

    }
}