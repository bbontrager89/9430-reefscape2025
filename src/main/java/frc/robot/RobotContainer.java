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
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightControlSubSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

        private final LightControlSubSystem lightControlSubSystem = new LightControlSubSystem();

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        // The operator's controller
        XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

        Double driverPOVRecency = null;
        int driverLatestPOVButton = -1;

        Double operatorPOVRecency = null;
        int operatorLatestPOVButton = -1;

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
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {

                /* * * * * * * * * * * * * *\
                 *                         *
                 * OPERATOR BUTTON MAPPING *
                 *                         *
                \* * * * * * * * * * * * * */

                // Right bumper - Coral manipulator wheels intake
                new JoystickButton(m_operatorController, Button.kRightBumper.value)
                                .onTrue((new InstantCommand()));

                // Left bumper - Coral manipulator wheels out
                new JoystickButton(m_operatorController, Button.kLeftBumper.value)
                                .onTrue((new InstantCommand()));

                // Y button - Toggle Coral Mode
                new JoystickButton(m_operatorController, Button.kY.value)
                                .onTrue((new InstantCommand()));

                // X button - Algae Reef Clear Mode
                new JoystickButton(m_operatorController, Button.kX.value)
                                .onTrue((new InstantCommand()));

                // B button - Algae intake mode
                new JoystickButton(m_operatorController, Button.kB.value)
                                .onTrue((new InstantCommand(new Runnable() {
                                        @Override
                                        public void run() {
                                                lightControlSubSystem.fastFlickerFor(2.5);
                                        }
                                })));

                // A button - Algae intake mode
                new JoystickButton(m_operatorController, Button.kA.value)
                                .onTrue((new InstantCommand(new Runnable() {
                                        @Override
                                        public void run() {
                                                lightControlSubSystem.turnOnFor(2.5);
                                        }
                                })));

                // Right Stick button - Transit mode
                new JoystickButton(m_operatorController, Button.kRightStick.value)
                                .onTrue((new InstantCommand()));

                // Left Stick button -
                new JoystickButton(m_operatorController, Button.kLeftStick.value)
                                .onTrue((new InstantCommand()));

                // Dpad Up button -
                new POVButton(m_operatorController, 0)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press -
                                                        // Coral mode: intake from Coral station
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 0;
                                        }
                                })));

                // Dpad Up-Right button -
                new POVButton(m_operatorController, 45)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 45;
                                        }
                                })));

                // Dpad Right button -
                new POVButton(m_operatorController, 90)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press -
                                                        // Coral mode: after down press: score L2 right
                                                        // Coral mode: after up press: score L3 right
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 90;
                                        }
                                })));

                // Dpad Down-Right button -
                new POVButton(m_operatorController, 135)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 135;
                                        }
                                })));

                // Dpad Down button -
                new POVButton(m_operatorController, 180)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press - Coral mode: after down press: score L1
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 180;
                                        }
                                })));

                // Dpad Down-Left button -
                new POVButton(m_operatorController, 225)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 225;
                                        }
                                })));

                // Dpad Left button -
                new POVButton(m_operatorController, 270)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press -
                                                        // Coral mode: after down press: score L2 left
                                                        // Coral mode: after up press: score L3 left
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 270;
                                        }
                                })));

                // Dpad Up-Left button -
                new POVButton(m_operatorController, 315)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (operatorPOVRecency != null && operatorPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                operatorPOVRecency = Timer.getFPGATimestamp();
                                                operatorLatestPOVButton = 315;
                                        }
                                })));

                // Start Button button - Manual mode on 2 second hold
                new JoystickButton(m_operatorController, Button.kStart.value)
                                .onTrue((new InstantCommand()));

                // Back Button button - Cancel all actions?
                new JoystickButton(m_operatorController, Button.kBack.value)
                                .onTrue((new InstantCommand()));

                /* * * * * * * * * * * * *\
                 *                       *
                 * DRIVER BUTTON MAPPING *
                 *                       *
                 * * * * * * * * * * * * */

                // Right bumper -
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .onTrue((new InstantCommand()));

                // Left bumper -
                new JoystickButton(m_driverController, Button.kLeftBumper.value)
                                .onTrue((new InstantCommand()));

                // Y button -
                new JoystickButton(m_driverController, Button.kY.value)
                                .onTrue((new InstantCommand()));

                // X button -
                new JoystickButton(m_driverController, Button.kX.value)
                                .onTrue((new InstantCommand()));

                // B button -
                new JoystickButton(m_driverController, Button.kB.value)
                                .onTrue((new InstantCommand()));

                // A button -
                new JoystickButton(m_driverController, Button.kA.value)
                                .onTrue((new InstantCommand()));

                // Right Stick button -
                new JoystickButton(m_driverController, Button.kRightStick.value)
                                .onTrue((new InstantCommand()));

                // Left Stick button -
                new JoystickButton(m_driverController, Button.kLeftStick.value)
                                .onTrue((new InstantCommand()));

                // Dpad Up button -
                new POVButton(m_driverController, 0)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 0;
                                        }
                                })));

                // Dpad Up-Right button -
                new POVButton(m_driverController, 45)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 45;
                                        }
                                })));

                // Dpad Right button -
                new POVButton(m_driverController, 90)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 90;
                                        }
                                })));

                // Dpad Down-Right button -
                new POVButton(m_driverController, 135)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 135;
                                        }
                                })));

                // Dpad Down button -
                new POVButton(m_driverController, 180)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 180;
                                        }
                                })));

                // Dpad Down-Left button -
                new POVButton(m_driverController, 225)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 225;
                                        }
                                })));

                // Dpad Left button -
                new POVButton(m_driverController, 270)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 270;
                                        }
                                })));

                // Dpad Up-Left button -
                new POVButton(m_driverController, 315)
                                .onTrue((new InstantCommand(new Runnable() {

                                        @Override
                                        public void run() {
                                                if (driverPOVRecency != null && driverPOVRecency + 0.25 > Timer
                                                                .getFPGATimestamp()) {
                                                        // on Double Press
                                                } else {
                                                        // on Single Press
                                                }

                                                driverPOVRecency = Timer.getFPGATimestamp();
                                                driverLatestPOVButton = 315;
                                        }
                                })));

                // Start Button button -
                new JoystickButton(m_driverController, Button.kStart.value)
                                .onTrue((new InstantCommand()));

                // Back Button button -
                new JoystickButton(m_driverController, Button.kBack.value)
                                .onTrue((new InstantCommand()));

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
