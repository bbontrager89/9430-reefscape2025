package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DoScorePositionCommand extends SequentialCommandGroup {
    private final DriveSubsystem drive;
    private final double desiredLateralOffset;
    private final double desiredDistance;

    public DoScorePositionCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coralSubsystem, DriveSubsystem drive, int scoringPosition, double desiredLateralOffset, double desiredDistance, double pivotHeight) {
        this.drive = drive;
        this.desiredLateralOffset = desiredLateralOffset;
        this.desiredDistance = desiredDistance;
        
        System.out.printf("ElevatorCommand created - Target lateral offset: %.2f m, Target distance: %.2f m%n",
            desiredLateralOffset, desiredDistance);

        // Only proceed if we initially see a tag
        addRequirements(drive, elevator);
        
        addCommands(
            new ConditionalCommand(
                // If we see a tag, execute the full alignment sequence
                new SequentialCommandGroup(
                    new RotateToTagCommand(drive),
                    new StrafeToAlignCommand(drive, desiredLateralOffset),
                    new MoveElevator(elevator, scoringPosition),
                    new PivotCoral(coralSubsystem, pivotHeight), // TODO make correct
                    new ApproachTagCommand(drive, desiredDistance, desiredLateralOffset),
                    new SetCoralSpeed(coralSubsystem, 1),
                    new WaitCommand(0.5),
                    new SetCoralSpeed(coralSubsystem, 0)
                ),
                // If we don't see a tag, do nothing
                new InstantCommand(),
                () -> drive.getPoseEstimatorSubsystem().getLastDetectedTagId() != -1
            )
        );
    }
}