// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransitModeCommand extends SequentialCommandGroup {

  /** Creates a new TransitModeCommand. */
  public TransitModeCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral);

    addCommands(
      new SequentialCommandGroup(
        // new PivotAlgae(algae, 0)
        new MoveElevator(elevator, 4), 
        new PivotCoral(coral, 0)));
  }
}
