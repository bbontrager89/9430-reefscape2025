// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
// import frc.utils.Elastic;

/** Subsystem for interfacing with motors and logic for the Robot's elevator */
public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex elevatorMotor = new SparkFlex(ElevatorConstants.elevatorMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder absoluteEncoder = elevatorMotor.getAbsoluteEncoder();

  private SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

  private PIDController elevatorController;

  private Double desiredHeight = null;
  private double autoSpeed = 0.0;

  private boolean aboveMaxHeight = false;
  private boolean belowMinHeight = false;
  private double currentSpeed = 0.0;

  private static SendableChooser<Command> elevatorCommands;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    elevatorMotorConfig.inverted(ElevatorConstants.elevatorMotorInverted);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    elevatorController.setTolerance(ElevatorConstants.positionTolerence);

    configureDashboardControls();

  }

  /** Puts commands on smartdashboard */
  public void configureDashboardControls() {

    elevatorCommands = new SendableChooser<Command>();

    elevatorCommands.addOption("SP 0", new InstantCommand(() -> {
      moveToScoringPosition(0);
    }));
    elevatorCommands.setDefaultOption("SP 1", new InstantCommand(() -> {
      moveToScoringPosition(1);
    }));
    elevatorCommands.addOption("SP 2", new InstantCommand(() -> {
      moveToScoringPosition(2);
    }));
    elevatorCommands.addOption("SP 3", new InstantCommand(() -> {
      moveToScoringPosition(3);
    }));
    elevatorCommands.addOption("min", new InstantCommand(() -> {
      moveToScoringPosition(4);
    }));
    elevatorCommands.addOption("max", new InstantCommand(() -> {
      moveToScoringPosition(5);
    }));
    elevatorCommands.addOption("Custom", new InstantCommand(() -> {
      moveToPosition(SmartDashboard.getNumber("Custom Elevator Height", ElevatorConstants.level1ScoringPosition));
    }));

    SmartDashboard.putData("Elevator Height Commands", elevatorCommands);

    SmartDashboard.putData("Run Elevator Command", new ScheduleCommand(new InstantCommand(() -> {
      if (elevatorCommands.getSelected() != null)
        elevatorCommands.getSelected().schedule();
    })));

    SmartDashboard.putNumber("Custom Elevator Height", ElevatorConstants.level1ScoringPosition);
    SmartDashboard.putNumber("Desired Height", 0.0);

  }

  /**
   * Sets the speed of the elevator motor
   * 
   * @param speed speed of the motor
   */
  public void setMotorSpeed(double speed) {
    checkHeight();
    currentSpeed = speed;
    if ((aboveMaxHeight && (currentSpeed > 0)) || 
        (belowMinHeight && (currentSpeed < 0))) {
      stopMotor();
    } else {
      elevatorMotor.set(currentSpeed);
    }
  }

  /** Stops elevator movement */
  public void stopMotor() {
    currentSpeed = 0.0;
    elevatorMotor.stopMotor();
  }

  /**
   * Sets the desired height for the PID controller given index
   * 
   * @param scoringPosition the index that relates to scoring position
   */
  public void moveToScoringPosition(int scoringPosition) {
    elevatorController.reset();

    switch (scoringPosition) {
      case 0:
        desiredHeight = ElevatorConstants.coralStationPosition;
        break;
      case 1:
        desiredHeight = ElevatorConstants.level1ScoringPosition;
        break;
      case 2:
        desiredHeight = ElevatorConstants.level2ScoringPosition;
        break;
      case 3:
        desiredHeight = ElevatorConstants.level3ScoringPosition;
        break;
      case 4:
        desiredHeight = ElevatorConstants.minimumElevatorHeight + 0.001;
        break;
      case 5:
        desiredHeight = ElevatorConstants.maximumElevatorHeight;
        break;
      default:
        System.out.println("Invalid Scoring Position requested in ElevatorSubsystem");
        // Elastic.sendError("Invalid Scoring Position", "requested in
        // ElevatorSubsystem");
        break;
    }

    SmartDashboard.putNumber("Desired Height", desiredHeight);

    // ;;;;;;;;desiredHeight=(scoringPosition==0)?ElevatorConstants.coralStationPosition:desiredHeight=(scoringPosition==1)?ElevatorConstants.level1ScoringPosition:(scoringPosition==2)?ElevatorConstants.level3ScoringPosition:(scoringPosition==3)?ElevatorConstants.level3ScoringPosition/*IWouldNotRecommend-Titus(ButItIsFunny)*/:(scoringPosition==4)?ElevatorConstants.level4ScoringPosition:-1.0;;;;;;;;;;;;;;;;;;;;;;;;;;;

  }

  /**
   * Calls moveToScoringPosition with the index of given SP
   * 
   * @param sp the desired scoring position
   */
  public void moveToScoringPosition(SP sp) {
    moveToScoringPosition(sp.index);
  }

  /**
   * Sets the desired elevator height position to given double
   * 
   * @param position the absolute encoder reading of the desired position
   */
  public void moveToPosition(double position) {
    elevatorController.reset();

    // Ensure that the desired postion is within bounds of the elevator
    desiredHeight = Math.min(Math.max(position, ElevatorConstants.minimumElevatorHeight),
        ElevatorConstants.maximumElevatorHeight);

    SmartDashboard.putNumber("Desired Height", desiredHeight);

  }

  /** Disable autonomous PID movement */
  public void turnOffAutoMode() {
    desiredHeight = null;
    stopMotor();
  }

  /**
   * Gets the double representing the absolute encoder postion of the elevator
   * 
   * @return absolute encoder reading
   */
  public double getHeight() {
    return absoluteEncoder.getPosition();
  }

  /**
   * Boolean representing if the elevator height is within
   * the tolerence threshold of the desired height for autonomous PID movement
   * 
   * @return boolean
   */
  public boolean atHeight() {
    return (Math.abs(getHeight() - desiredHeight) < ElevatorConstants.positionTolerence);
  }

  public void checkHeight() {
    // Lower soft limit check
    if (getHeight() > ElevatorConstants.maximumElevatorHeight) {
      if (currentSpeed > 0) {
        // stopMotor();
      }
      if (autoSpeed > 0 && desiredHeight != null) {
        // turnOffAutoMode();
      }
      aboveMaxHeight = true;
    } else {
      aboveMaxHeight = false;
    }

    // Upper soft limit check
    if (getHeight() < ElevatorConstants.minimumElevatorHeight) {
      if (currentSpeed < 0) {
        stopMotor();
      }

      if (autoSpeed < 0 && desiredHeight != null) {
        turnOffAutoMode();
      }

      belowMinHeight = true;

    } else {
      belowMinHeight = false;
    }
  }

  @Override
  public void periodic() {

    // Log data
    SmartDashboard.putNumber("Elevator Position", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", absoluteEncoder.getVelocity());

    checkHeight();


    // Run auto movement
    if (desiredHeight != null) {

      // Calculate error
      // double elevatorError = (desiredHeight - getHeight());

      /*
       * // Adjust speed to error
       * double autoSpeed = ElevatorConstants.kP * elevatorError;
       * autoSpeed = (autoSpeed > ElevatorConstants.maximumAutoSpeed) ?
       * ElevatorConstants.maximumAutoSpeed
       * : (autoSpeed < -ElevatorConstants.maximumAutoSpeed) ?
       * -ElevatorConstants.maximumAutoSpeed : autoSpeed;
       * 
       */

      autoSpeed = elevatorController.calculate(getHeight(), desiredHeight);
      setMotorSpeed(autoSpeed);

      // Log data
      SmartDashboard.putNumber("AutoSpeed", autoSpeed);

    }

  }

  /**
   * Enum usful for readability for <strong>Scoring Position</strong> elevator
   * movement calls
   */
  public static enum SP {
    /** SP1 */
    one(1),
    /** SP2 */
    two(2),
    /** SP3 */
    three(3),
    /** The minimum elevator height */
    min(4),
    /** The maximum elevator height */
    max(5);

    /** The index that the {@Link ElevatorSubsystem} uses */
    public final int index;

    SP(int index) {
      this.index = index;
    }
  }
}
