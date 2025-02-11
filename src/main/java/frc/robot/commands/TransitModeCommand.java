// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransitModeCommand extends Command {

  ElevatorSubsystem elevator;
  CoralManipulatorSubsystem coral;

  /** Creates a new TransitModeCommand. */
  public TransitModeCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral);

    this.elevator = elevator;
    this.coral = coral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.moveToScoringPosition(0);
    coral.movePivotTo(CoralManipulatorConstants.maximumPivotPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atHeight() && coral.atPivotPosition();
  }
}
