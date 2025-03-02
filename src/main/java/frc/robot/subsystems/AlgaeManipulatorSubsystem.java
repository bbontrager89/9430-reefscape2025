// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

  private SparkFlex intakeMotor = new SparkFlex(AlgaeConstants.intakeMotorCANid, MotorType.kBrushless);
  private SparkFlex pivotMotor = new SparkFlex(AlgaeConstants.pivotMotorCANid, MotorType.kBrushless);

  private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private Double desiredPivotHeight = null;

  private boolean aboveMaxHeight = false;
  private boolean belowMinHeight = false;

  private double pivotSpeed = 0.0;

  /** Creates a new AlgaeManipulatorSubSystem. */
  public AlgaeManipulatorSubsystem() {}

  /**
   * Sets the speed of the intake motor
   * @param speed
   */
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Stops the intake motor
   */
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  /**
   * Sets the speed of the pivot motor
   * <p>
   * <b>Don't manully use without first running {@link AlgaeManipulatorSubsystem#disableAutoPivot()} </b>
   * @param speed
   */
  public void setPivotSpeed(double speed) {
    if (!(aboveMaxHeight && speed < 0) && !(belowMinHeight && speed > 0)) {
      pivotMotor.set(speed);
      pivotSpeed = speed;
    }
  }

  /**
   * Stops the pivot motor
   */
  public void stopPivot() {
    pivotMotor.stopMotor();
    pivotSpeed = 0.0;
  }

  /**
   * Sets the disired height of the pivot
   * @param height double representing the height
   */
  public void setDesiredPivotHeight(double height) {
    desiredPivotHeight = 
      Math.min(Math.max(height, 
        AlgaeConstants.minimumPivotPosition),
        AlgaeConstants.maximumPivotPosition);
  }

  /**
   * Sets the disired height of the pivot
   * uses enum {@link AP} for setting the position
   * @param ap the Algae Position
   */
  public void setDesiredPivotHeight(AP ap) {
    switch (ap) {
      case intaking:
        desiredPivotHeight = AlgaeConstants.intakeHeight;
        break;

      case transit:
        desiredPivotHeight = AlgaeConstants.transitHeight;
        break;

      case minimum:
        desiredPivotHeight = AlgaeConstants.minimumPivotPosition;
        break;

      case maximum:
        desiredPivotHeight = AlgaeConstants.maximumPivotPosition;
        break;
    
      default:
        desiredPivotHeight = null;
        break;
    };
  }

  /**
   * Disables auto pivot movment and stops motor
   */
  public void disableAutoPivot() {
    desiredPivotHeight = null;
    stopPivot();
  }

  /**
   * The absolute encoder reading of the pivot motor
   * @return double representing position
   */
  public double getPivotHeight() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Lower soft limit check
    if (getPivotHeight() > AlgaeConstants.maximumPivotPosition) {
      if (pivotSpeed < 0) {
        stopPivot();
      }

      aboveMaxHeight = true;

    } else {
      aboveMaxHeight = false;
    }

    // Upper soft limit check
    if (getPivotHeight() < AlgaeConstants.minimumPivotPosition) {
      if (pivotSpeed > 0) {
        stopPivot();
      }

      belowMinHeight = true;

    } else {
      belowMinHeight = false;
    }

    if (desiredPivotHeight != null) {
      // PID controller to position

      // Calculate error
      double pivotError = (desiredPivotHeight - getPivotHeight());

      // Adjust speed to error
      double autoSpeed = AlgaeConstants.kP * pivotError;
      autoSpeed = (autoSpeed > AlgaeConstants.maximumAutoSpeed) ? AlgaeConstants.maximumAutoSpeed
          : (autoSpeed < -AlgaeConstants.maximumAutoSpeed) ? -AlgaeConstants.maximumAutoSpeed : autoSpeed;

      // Log data
      SmartDashboard.putNumber("Algae Pivot Error", pivotError);

      // Run or don't run
      if (Math.abs(pivotError) < AlgaeConstants.pivotTolerence || Math.abs(autoSpeed) < 0.001) {
        disableAutoPivot();
      } else {
        setPivotSpeed(autoSpeed);
      }
    }

  }

  /** <B>Algae Position</B> enum for readability */
  public static enum AP {
    maximum(0),
    minimum(1),
    transit(2),
    intaking(3);

    public final int index;

    AP(int index) {
      this.index = index;
    }
  }
}
