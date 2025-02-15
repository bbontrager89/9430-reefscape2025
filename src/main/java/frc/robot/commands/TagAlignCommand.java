package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class TagAlignCommand extends SequentialCommandGroup {
    private final DriveSubsystem drive;
    private final double desiredLateralOffset;
    private final double desiredDistance;

    public TagAlignCommand(DriveSubsystem drive, double desiredLateralOffset, double desiredDistance) {
        this.drive = drive;
        this.desiredLateralOffset = desiredLateralOffset;
        this.desiredDistance = desiredDistance;
        
        System.out.printf("TagAlignCommand created - Target lateral offset: %.2f m, Target distance: %.2f m%n",
            desiredLateralOffset, desiredDistance);

        // Only proceed if we initially see a tag
        addRequirements(drive);
        
        addCommands(
            new ConditionalCommand(
                // If we see a tag, execute the full alignment sequence
                new SequentialCommandGroup(
                    new RotateToTagCommand(drive),
                    new StrafeToAlignCommand(drive, desiredLateralOffset),
                    new ApproachTagCommand(drive, desiredDistance)
                ),
                // If we don't see a tag, do nothing
                new InstantCommand(),
                () -> drive.getPoseEstimatorSubsystem().getLastDetectedTagId() != -1
            )
        );
    }
}