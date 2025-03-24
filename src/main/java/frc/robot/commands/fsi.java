// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class fsi extends Command {
  CoralManipulatorSubsystem coral;
  ElevatorSubsystem elevator;
  /** Creates a new fsi. */
  public fsi(CoralManipulatorSubsystem coral, ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral, elevator);
    this.coral = coral;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.moveToPosition(
                                                                                ElevatorConstants.coralStationPosition);
                                                                coral.movePivotTo(
                                                                                CoralManipulatorConstants.intakePivotPosition);
                                                                  coral.startIntakeMotor(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coral.isCoralIntaken();
  }
}
