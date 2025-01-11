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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
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

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // The driver's controller
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    Double povPressedRecency = null;
    int latestPOVButtonPress = -1;

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
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
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

        // Right Trigger - Coral manipulator wheels intake
        new JoystickButton(m_driverController, edu.wpi.first.wpilibj.PS4Controller.Button.kR1.value)
        /* .whileTrue(new RunCommand()) */;

        // Left Trigger - Coral manipulator wheels out
        new JoystickButton(m_driverController, edu.wpi.first.wpilibj.PS4Controller.Button.kL1.value)
        /* .whileTrue(new RunCommand()) */;

        // Y button - Coral mode
        new JoystickButton(m_operatorController, Button.kY.value)
                .onTrue((new InstantCommand()));

        // X button - Algae clear from reef mode
        new JoystickButton(m_operatorController, Button.kX.value)
                .onTrue((new InstantCommand()));

        // B button - Algae intake mode
        new JoystickButton(m_operatorController, Button.kB.value)
                .onTrue((new InstantCommand()));

        // A button - Algae intake mode
        new JoystickButton(m_operatorController, Button.kA.value)
                .onTrue((new InstantCommand()));

        // Right Stick button - Transit mode
        new JoystickButton(m_operatorController, Button.kRightStick.value)
                .onTrue((new InstantCommand()));

        // Left Stick button - unbound?
        new JoystickButton(m_operatorController, Button.kLeftStick.value)
                .onTrue((new InstantCommand()));

        // Dpad Up button
        new POVButton(m_driverController, 0)
                .onTrue((new InstantCommand(new Runnable() {

                    @Override
                    public void run() {
                        if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                                // on Double Press
                        } else {
                                // on Single Press
                        }

                        povPressedRecency = Timer.getFPGATimestamp();
                        latestPOVButtonPress = 0;
                    }
                })));

        // Dpad Up-Right button
        new POVButton(m_operatorController, 45)
                .onTrue((new InstantCommand(new Runnable() {

                    @Override
                    public void run() {
                        if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                                // on Double Press
                        } else {
                                // on Single Press
                        }
                        
                        povPressedRecency = Timer.getFPGATimestamp();
                        latestPOVButtonPress = 45;
                    }
                })));

        // Dpad Right button
        new POVButton(m_operatorController, 90)
        .onTrue((new InstantCommand(new Runnable() {

                @Override
                public void run() {
                    if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                            // on Double Press
                    } else {
                            // on Single Press
                    }
                    
                    povPressedRecency = Timer.getFPGATimestamp();
                    latestPOVButtonPress = 90;
                }
            })));

        // Dpad Down-Right button
        new POVButton(m_operatorController, 135)
        .onTrue((new InstantCommand(new Runnable() {

                @Override
                public void run() {
                    if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                            // on Double Press
                    } else {
                            // on Single Press
                    }
                    
                    povPressedRecency = Timer.getFPGATimestamp();
                    latestPOVButtonPress = 135;
                }
            })));

        // Dpad Down button
        new POVButton(m_operatorController, 180)
        .onTrue((new InstantCommand(new Runnable() {

                @Override
                public void run() {
                    if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                            // on Double Press
                    } else {
                            // on Single Press
                    }
                    
                    povPressedRecency = Timer.getFPGATimestamp();
                    latestPOVButtonPress = 180;
                }
            })));

        // Dpad Down-Left button
        new POVButton(m_operatorController, 225)
        .onTrue((new InstantCommand(new Runnable() {

                @Override
                public void run() {
                    if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                            // on Double Press
                    } else {
                            // on Single Press
                    }
                    
                    povPressedRecency = Timer.getFPGATimestamp();
                    latestPOVButtonPress = 225;
                }
            })));

        // Dpad Left button
        new POVButton(m_operatorController, 270)
        .onTrue((new InstantCommand(new Runnable() {

                @Override
                public void run() {
                    if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                            // on Double Press
                    } else {
                            // on Single Press
                    }
                    
                    povPressedRecency = Timer.getFPGATimestamp();
                    latestPOVButtonPress = 270;
                }
            })));

        // Dpad Up-Left button
        new POVButton(m_operatorController, 315)
        .onTrue((new InstantCommand(new Runnable() {

                @Override
                public void run() {
                    if (povPressedRecency != null && povPressedRecency + 0.25 > Timer.getFPGATimestamp()) {
                            // on Double Press
                    } else {
                            // on Single Press
                    }
                    
                    povPressedRecency = Timer.getFPGATimestamp();
                    latestPOVButtonPress = 315;
                }
            })));

        // Start Button button - Manual mode
        new JoystickButton(m_operatorController, Button.kStart.value)
                .onTrue((new InstantCommand()));

        // Back Button button - Cancel all actions?
        new JoystickButton(m_operatorController, Button.kBack.value)
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
