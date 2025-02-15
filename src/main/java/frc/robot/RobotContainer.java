// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DoScorePositionCommand;
import frc.robot.commands.TransitModeCommand;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.ControllerUtils.POV;
import frc.utils.ControllerUtils.AXIS;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();

        private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

        private CoralManipulatorSubsystem coralManipulatorSubsystem = new CoralManipulatorSubsystem();


        // The driver's controller
        CommandXboxController c_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        // The operator's controller
        CommandXboxController c_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(c_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(c_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(c_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));
        }

        /** Represents modes for different controls */
        enum ControlMode {
                /**
                 * <p>
                 * Left stick moves elevator up and down
                 * <p>
                 * Right stick moves coral manipulator up and down
                 * <p>
                 * D-Pad right pivots algae intake out
                 * <p>
                 * D-Pad left pivots algae intake in
                 * <p>
                 * RB hold - Algae intake wheels spin out
                 * <p>
                 * LB hold - Algae intake wheels spin in
                 * <p>
                 * RT hold - Coral manipulator wheels intake
                 * <p>
                 * LT hold - Coral manipulator wheels out
                 */
                Manual,
                /** Robot is controlled by commands mapped to button bindings */
                SemiAuto
        }

        Double driverPOVRecency = null;
        POV driverLatestPOVButton = POV.None;

        Double operatorPOVRecency = null;
        POV operatorLatestPOVButton = POV.None;

        ControlMode activeMode = ControlMode.Manual;

        double operatorStartButtonTimestamp = Double.NEGATIVE_INFINITY;

        /**
         * ~~Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} 
         * or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or 
         * {@link XboxController}), and then calling passing it to a
         * {@link JoystickButton}.~~
         * 
         * <p>
         * Binds Commands to Xbox controller buttons using 
         * {@link CommandXboxController} methods
         * <p>
         * This method should only be run once by the constructer
         */
        private void configureButtonBindings() {

                /* * * * * * * * * * * * * *\
                 *                         *
                 * OPERATOR BUTTON MAPPING *
                 *                         *
                \* * * * * * * * * * * * * */

                // Left Stick Vetical Absolute Value Greater Than Threshold
                c_operatorController.axisGreaterThan(AXIS.LeftVertical.value, OIConstants.kTriggerThreshold).or(
                c_operatorController.axisLessThan(AXIS.LeftVertical.value, -OIConstants.kTriggerThreshold))
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                if (activeMode == ControlMode.Manual) {
                                      elevatorSubsystem.setMotorSpeed(c_operatorController.getLeftY());  
                                }
                        }))).onFalse(new InstantCommand(() -> {
                                elevatorSubsystem.setMotorSpeed(0); 
                        }));

                // Left Stick Horizontal Absolute Value Greater Than Threshold
                c_operatorController.axisGreaterThan(AXIS.LeftHorizontal.value, OIConstants.kTriggerThreshold).or(
                c_operatorController.axisLessThan(AXIS.LeftHorizontal.value, -OIConstants.kTriggerThreshold))
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                if (activeMode == ControlMode.Manual) {
                                        
                                }
                        }))).onFalse(new InstantCommand(() -> {

                        }));

                // Right Stick Vetical Absolute Value Greater Than Threshold
                c_operatorController.axisGreaterThan(AXIS.RightVertical.value, OIConstants.kTriggerThreshold).or(
                c_operatorController.axisLessThan(AXIS.RightVertical.value, -OIConstants.kTriggerThreshold))
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                if (activeMode == ControlMode.Manual) {
                                        
                                }
                        }))).onFalse(new InstantCommand(() -> {

                        }));

                // Right Stick Horizontal Absolute Value Greater Than Threshold
                c_operatorController.axisGreaterThan(AXIS.RightHorizontal.value, OIConstants.kTriggerThreshold).or(
                c_operatorController.axisLessThan(AXIS.RightHorizontal.value, -OIConstants.kTriggerThreshold))
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                if (activeMode == ControlMode.Manual) {
                                        
                                }
                        }))).onFalse(new InstantCommand(() -> {

                        }));
                

                // Right bumper - Manual mode: Coral manipulator wheels intake
                c_operatorController.rightBumper()
                        .onTrue(new InstantCommand(() -> {
                                coralManipulatorSubsystem.movePivotTo(coralManipulatorSubsystem.getPivotMotorPosition() - 0.05);
                        }));

                // Right trigger -
                c_operatorController.rightTrigger(OIConstants.kTriggerThreshold)
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                coralManipulatorSubsystem.startIntakeMotor(-1 * c_operatorController.getRightTriggerAxis());

                        }))).onFalse(new InstantCommand(() -> {
                                coralManipulatorSubsystem.stopIntakeMotor();
                        }));

                // Left bumper - Coral manipulator wheels out
                c_operatorController.leftBumper()
                        .onTrue(new InstantCommand(() -> {
                                coralManipulatorSubsystem.movePivotTo(coralManipulatorSubsystem.getPivotMotorPosition() + 0.05);
                        }));

                // Left trigger -
                c_operatorController.leftTrigger(OIConstants.kTriggerThreshold)
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                coralManipulatorSubsystem.startIntakeMotor(c_operatorController.getLeftTriggerAxis());

                        }))).onFalse(new InstantCommand(() -> {
                                coralManipulatorSubsystem.stopIntakeMotor();
                        }));

                // Y button - Toggle Coral Mode
                c_operatorController.y()
                        .whileTrue(new InstantCommand(() -> {
                                if (c_operatorController.getLeftY() > 0.1)
                                        elevatorSubsystem.setMotorSpeed(-c_operatorController.getLeftY());
                                else 
                                        elevatorSubsystem.setMotorSpeed(0);

                        })).onFalse(new InstantCommand(() -> {
                                elevatorSubsystem.setMotorSpeed(0);
                        }));

                // X button - Algae Reef Clear Mode
                c_operatorController.x()
                        .onTrue(new InstantCommand());

                // B button - Algae intake mode
                c_operatorController.b()
                        .onTrue(new InstantCommand(() -> {
                                

                        }));

                // A button - Algae intake mode
                c_operatorController.a()
                        .onTrue(new InstantCommand(() -> {
                                

                        }));

                // Right Stick button - Transit mode
                c_operatorController.rightStick()
                        .onTrue(new TransitModeCommand(elevatorSubsystem, coralManipulatorSubsystem));

                // Left Stick button -
                c_operatorController.leftStick()
                        .onTrue(new InstantCommand());

                // Dpad Up button -
                c_operatorController.povUp()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press -
                                        // Coral mode: intake from Coral station
                                        if (operatorLatestPOVButton == POV.Up) {
                                                new DoScorePositionCommand(
                                                        elevatorSubsystem, 
                                                        coralManipulatorSubsystem, 
                                                        m_robotDrive, 
                                                        0,
                                                        0, 
                                                        0.0, 
                                                        OIConstants.scoringDistance, 
                                                        CoralManipulatorConstants.levelThreePivotPosition)
                                                .schedule();
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Up;
                        }));

                // Dpad Up-Right button -
                c_operatorController.povUpRight()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.UpRight;
                        }));

                // Dpad Right button -
                c_operatorController.povRight()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press -
                                        // Coral mode: after down press: score L2 right
                                        // Coral mode: after up press: score L3 right
                                        if (operatorLatestPOVButton == POV.Down) {
                                                new DoScorePositionCommand(
                                                        elevatorSubsystem, 
                                                        coralManipulatorSubsystem, 
                                                        m_robotDrive, 
                                                        0,
                                                        2, 
                                                        OIConstants.rightScoringOffset, 
                                                        OIConstants.scoringDistance, 
                                                        CoralManipulatorConstants.levelTwoPivotPosition)
                                                .schedule();
                                        }
                                        if (operatorLatestPOVButton == POV.Up) {
                                                new DoScorePositionCommand(
                                                        elevatorSubsystem, 
                                                        coralManipulatorSubsystem, 
                                                        m_robotDrive, 
                                                        0,
                                                        3, 
                                                        OIConstants.rightScoringOffset, 
                                                        OIConstants.scoringDistance, 
                                                        CoralManipulatorConstants.levelThreePivotPosition)
                                                .schedule();
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Right;
                        }));

                // Dpad Down-Right button -
                c_operatorController.povDownRight()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.DownRight;
                        }));

                // Dpad Down button -
                c_operatorController.povDown()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press - Coral mode: after down press: score L1
                                        if (operatorLatestPOVButton == POV.Down) {
                                                new DoScorePositionCommand(
                                                        elevatorSubsystem, 
                                                        coralManipulatorSubsystem, 
                                                        m_robotDrive, 
                                                        0,
                                                        1, 
                                                        0.0, 
                                                        OIConstants.scoringDistance, 
                                                        CoralManipulatorConstants.levelOnePivotPosition)
                                                .schedule();
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Down;
                        }));

                // Dpad Down-Left button -
                c_operatorController.povDownLeft()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.DownLeft;
                        }));

                // Dpad Left button -
                c_operatorController.povLeft()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press -
                                        // Coral mode: after down press: score L2 left
                                        // Coral mode: after up press: score L3 left
                                        if (operatorLatestPOVButton == POV.Down) {
                                                new DoScorePositionCommand(
                                                        elevatorSubsystem, 
                                                        coralManipulatorSubsystem, 
                                                        m_robotDrive, 
                                                        0,
                                                        2, 
                                                        OIConstants.leftScoringOffset, 
                                                        OIConstants.scoringDistance, 
                                                        CoralManipulatorConstants.levelTwoPivotPosition)
                                                .schedule();
                                        }
                                        if (operatorLatestPOVButton == POV.Up) {
                                                new DoScorePositionCommand(
                                                        elevatorSubsystem, 
                                                        coralManipulatorSubsystem, 
                                                        m_robotDrive, 
                                                        0,
                                                        3, 
                                                        OIConstants.leftScoringOffset, 
                                                        OIConstants.scoringDistance, 
                                                        CoralManipulatorConstants.levelThreePivotPosition)
                                                .schedule();
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Left;
                        }));

                // Dpad Up-Left button -
                c_operatorController.povUpLeft()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.UpLeft;
                        }));

                // Start Button button - Manual mode on 2 second hold
                c_operatorController.start()
                        .onTrue(new InstantCommand(() -> {
                                operatorStartButtonTimestamp = Timer.getFPGATimestamp();

                        })).onFalse(new InstantCommand(() -> {
                                operatorStartButtonTimestamp = Double.NEGATIVE_INFINITY;

                        })).whileTrue(new InstantCommand(() -> {
                                if (operatorStartButtonTimestamp + 2 < Timer.getFPGATimestamp()) {
                                        if (activeMode == ControlMode.SemiAuto)
                                                activeMode = ControlMode.Manual;
                                        else 
                                                activeMode = ControlMode.SemiAuto;
                                }
                        }));

                // Back Button button - Cancel all actions?
                c_operatorController.back()
                        .onTrue(new InstantCommand());

                /* * * * * * * * * * * * *\
                 *                       *
                 * DRIVER BUTTON MAPPING *
                 *                       *
                 * * * * * * * * * * * * */

                // Right bumper -
                c_driverController.rightBumper()
                        .onTrue(new InstantCommand());

                // Right trigger -
                c_driverController.rightTrigger(OIConstants.kTriggerThreshold)
                        .onTrue(new InstantCommand());

                // Left bumper -
                c_driverController.leftBumper()
                        .onTrue(new InstantCommand());

                // Left trigger -
                c_driverController.leftTrigger(OIConstants.kTriggerThreshold)
                        .onTrue(new InstantCommand());

                // Y button -
                c_driverController.y()
                        .onTrue(new InstantCommand());

                // X button -
                c_driverController.x()
                        .onTrue(new InstantCommand());

                // B button -
                c_driverController.b()
                        .onTrue(new InstantCommand());

                // A button -
                c_driverController.a()
                        .onTrue(new DoScorePositionCommand(elevatorSubsystem, coralManipulatorSubsystem, m_robotDrive, 0, 3, 0.3, 1.5, 0.25));

                // Right Stick button -
                c_driverController.rightStick()
                        .onTrue(new InstantCommand());

                // Left Stick button -
                c_driverController.leftStick()
                        .onTrue(new InstantCommand());

                // Dpad Up button -
                c_driverController.povUp()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Up;
                        }));

                // Dpad Up-Right button -
                c_driverController.povUpRight()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.UpRight;
                        }));

                // Dpad Right button -
                c_driverController.povRight()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Right;
                        }));

                // Dpad Down-Right button -
                c_driverController.povDownRight()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.DownRight;
                        }));

                // Dpad Down button -
                c_driverController.povDown()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Down;
                        }));

                // Dpad Down-Left button -
                c_driverController.povDownLeft()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.DownLeft;
                        }));

                // Dpad Left button -
                c_driverController.povLeft()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Left;
                        }));

                // Dpad Up-Left button -
                c_driverController.povUpLeft()
                        .onTrue(new InstantCommand(() -> {
                                if (driverPOVRecency != null && 
                                        driverPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press
                                } else {
                                        // on Single Press
                                }

                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.UpLeft;
                        }));

                // Start Button button -
                c_driverController.start()
                        .onTrue(new InstantCommand());

                // Back Button button -
                c_driverController.back()
                        .onTrue(new InstantCommand());

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)),
                                config);

                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                // Reset odometry to the starting pose of the trajectory.
                m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        }
}
