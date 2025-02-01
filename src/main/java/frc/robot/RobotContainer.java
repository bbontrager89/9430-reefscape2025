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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.ControllerUtils.POV;

import java.util.List;

/*
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
        CommandXboxController c_driverController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        // The operator's controller
        CommandXboxController c_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        Double driverPOVRecency = null;
        POV driverLatestPOVButton = POV.None;

        Double operatorPOVRecency = null;
        POV operatorLatestPOVButton = POV.None;

        ControlMode activeMode = ControlMode.Manual;
        double intakeMotorSpeed = 0.3;


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
                 * Retract Algae intake
                 * <p>
                 * Coral Manipulator to upright position
                 * <p>
                 * Elevator to bottom position
                 */
                Transit,
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
         * This method should only be run once by the contructer
         */
        private void configureButtonBindings() {

                /* * * * * * * * * * * * * *\
                 *                         *
                 * OPERATOR BUTTON MAPPING *
                 *                         *
                \* * * * * * * * * * * * * */

                // Right bumper - Manual mode: Coral manipulator wheels intake
                c_operatorController.rightBumper()
                        .onTrue(new InstantCommand(new Runnable() {
                                @Override
                                public void run() {
                                        activeMode = ControlMode.Transit;
                                }
                        }));

                // Right trigger -
                c_operatorController.rightTrigger(OIConstants.kTriggerThreshold)
                        .onTrue(new InstantCommand(new Runnable() {
                                @Override
                                public void run() {
                                        coralManipulatorSubsystem.runIntakeFor(1, 2);
                                }
                        }));

                // Left bumper - Coral manipulator wheels out
                c_operatorController.leftBumper()
                        .onTrue(new InstantCommand());

                // Left trigger -
                c_operatorController.leftTrigger(OIConstants.kTriggerThreshold)
                        .onTrue(new InstantCommand());

                // Y button - Toggle Coral Mode
                c_operatorController.y()
                        .onTrue(new InstantCommand());

                // X button - Algae Reef Clear Mode
                c_operatorController.x()
                        .onTrue(new InstantCommand());

                // B button - Algae intake mode
                c_operatorController.b()
                        .onTrue(new InstantCommand());

                // A button - Algae intake mode
                c_operatorController.a()
                        .onTrue(new InstantCommand());

                // Right Stick button - Transit mode
                c_operatorController.rightStick()
                        .onTrue(new InstantCommand());

                // Left Stick button -
                c_operatorController.leftStick()
                        .onTrue(new InstantCommand());

                // Dpad Up button -
                c_operatorController.povUp()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press -
                                                // Coral mode: intake from Coral station
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.Up;
                                }
                        }));

                // Dpad Up-Right button -
                c_operatorController.povUpRight()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.UpRight;
                                }
                        }));

                // Dpad Right button -
                c_operatorController.povRight()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press -
                                                // Coral mode: after down press: score L2 right
                                                // Coral mode: after up press: score L3 right
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.Right;
                                }
                        }));

                // Dpad Down-Right button -
                c_operatorController.povDownRight()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.DownRight;
                                }
                        }));

                // Dpad Down button -
                c_operatorController.povDown()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press - Coral mode: after down press: score L1
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.Down;
                                }
                        }));

                // Dpad Down-Left button -
                c_operatorController.povDownLeft()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.DownLeft;
                                }
                        }));

                // Dpad Left button -
                c_operatorController.povLeft()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press -
                                                // Coral mode: after down press: score L2 left
                                                // Coral mode: after up press: score L3 left
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.Left;
                                }
                        }));

                // Dpad Up-Left button -
                c_operatorController.povUpLeft()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (operatorPOVRecency != null && 
                                                operatorPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        operatorPOVRecency = Timer.getFPGATimestamp();
                                        operatorLatestPOVButton = POV.UpLeft;
                                }
                        }));

                // Start Button button - Manual mode on 2 second hold
                c_operatorController.start()
                        .onTrue(new InstantCommand());

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
                        .onTrue(new InstantCommand());

                // Right Stick button -
                c_driverController.rightStick()
                        .onTrue(new InstantCommand());

                // Left Stick button -
                c_driverController.leftStick()
                        .onTrue(new InstantCommand());

                // Dpad Up button -
                c_driverController.povUp()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.Up;
                                }
                        }));

                // Dpad Up-Right button -
                c_driverController.povUpRight()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.UpRight;
                                }
                        }));

                // Dpad Right button -
                c_driverController.povRight()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.Right;
                                }
                        }));

                // Dpad Down-Right button -
                c_driverController.povDownRight()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.DownRight;
                                }
                        }));

                // Dpad Down button -
                c_driverController.povDown()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.Down;
                                }
                        }));

                // Dpad Down-Left button -
                c_driverController.povDownLeft()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.DownLeft;
                                }
                        }));

                // Dpad Left button -
                c_driverController.povLeft()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.Left;
                                }
                        }));

                // Dpad Up-Left button -
                c_driverController.povUpLeft()
                        .onTrue(new InstantCommand(new Runnable() {

                                @Override
                                public void run() {
                                        if (driverPOVRecency != null && 
                                                driverPOVRecency + 0.25 > Timer.getFPGATimestamp()) {
                                                // on Double Press
                                        } else {
                                                // on Single Press
                                        }

                                        driverPOVRecency = Timer.getFPGATimestamp();
                                        driverLatestPOVButton = POV.UpLeft;
                                }
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
