// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.SP;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClearAlgaeFromReef extends SequentialCommandGroup {
  /** Creates a new ClearAlgaeFromReef. */
  public ClearAlgaeFromReef(ElevatorSubsystem elevator, CoralManipulatorSubsystem coralSubsystem,
            DriveSubsystem drive, SP level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new ConditionalCommand(
                // If we see a tag, execute the full alignment sequence
                new SequentialCommandGroup(
                    new RotateToTagCommand(drive),
                    new StrafeToAlignCommand(drive, OIConstants.algaeOffset),
                    new MoveElevator(elevator, level.index),
                    new PivotCoral(coralSubsystem, CoralManipulatorConstants.algaeClear),
                    new ApproachTagCommand(drive, 0.15, OIConstants.algaeOffset),
                    new SetCoralSpeed(coralSubsystem, -1),
                    new InstantCommand(() -> {
                        drive.drive(0,0.2, 0, false);
                    }),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> {
                        drive.drive(0, 0, 0, false);
                    }),
                    new SetCoralSpeed(coralSubsystem, 0),
                    new TransitModeCommand(elevator, coralSubsystem)
                ),
                // If we don't see a tag, do nothing
                new InstantCommand(),
                () -> drive.getPoseEstimatorSubsystem().getLastDetectedTagId() != -1
            )
        );
  }
}
