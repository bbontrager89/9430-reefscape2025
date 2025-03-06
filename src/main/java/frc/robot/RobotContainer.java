// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DoScorePositionCommand;
import frc.robot.commands.DoIntakeCoralFromStationCommand;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.TransitModeCommand;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.ClimbingArmSubsystem;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeManipulatorSubsystem.AP;
import frc.robot.subsystems.ElevatorSubsystem.SP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.ControllerUtils.POV;
import frc.utils.ControllerUtils;
import frc.utils.ControllerUtils.AXIS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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

        private AlgaeManipulatorSubsystem algaeManipulatorSubsystem = new AlgaeManipulatorSubsystem();
        private final SendableChooser<Command> autoChooser;

        private ClimbingArmSubsystem climbingArmSubsystem = new ClimbingArmSubsystem();


        // The driver's controller
        public static CommandXboxController c_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        // The operator's controller
        public static CommandXboxController c_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure command Chooser
                configureNamedCommands();

                // Configure the button bindings
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser("2 Coral Auto "
                                + (DriverStation.getAlliance().get() == Alliance.Blue ? "Blue" : "Red"));

                SmartDashboard.putData("Auto Chooser", autoChooser);
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

        /** Configures NamedCommands for pathplanner  */
        
        /** Configures NamedCommands for pathplanner */
        private void configureNamedCommands() {

                // Register Intake Coral Station
                NamedCommands.registerCommand("Intake Coral Station",
                                new DoIntakeCoralFromStationCommand(
                                                elevatorSubsystem,
                                                coralManipulatorSubsystem,
                                                m_robotDrive));
                // Register Intake Coral Station
                NamedCommands.registerCommand("Transit Mode",
                                new TransitModeCommand(
                                                elevatorSubsystem,
                                                coralManipulatorSubsystem));
                                                
                // Register Score LP2 Left
                NamedCommands.registerCommand("Score LP2 Left",
                                new DoScorePositionCommand(
                                                elevatorSubsystem,
                                                coralManipulatorSubsystem,
                                                m_robotDrive,
                                                2, // Level 2 scoring position
                                                OIConstants.leftScoringOffset,
                                                OIConstants.scoringDistanceLeft,
                                                CoralManipulatorConstants.levelTwoPivotPosition));

                // Register Score LP3 Left
                NamedCommands.registerCommand("Score LP3 Left",
                                new DoScorePositionCommand(
                                                elevatorSubsystem,
                                                coralManipulatorSubsystem,
                                                m_robotDrive,
                                                3, // Level 3 scoring position
                                                OIConstants.leftScoringOffset,
                                                OIConstants.scoringDistanceLeft,
                                                CoralManipulatorConstants.levelThreePivotPosition));

                // Register Score LP2 Right
                NamedCommands.registerCommand("Score LP2 Right",
                                new DoScorePositionCommand(
                                                elevatorSubsystem,
                                                coralManipulatorSubsystem,
                                                m_robotDrive,
                                                2, // Level 2 scoring position
                                                OIConstants.rightScoringOffset,
                                                OIConstants.scoringDistanceRight,
                                                CoralManipulatorConstants.levelTwoPivotPosition));

                // Register Score LP3 Right
                NamedCommands.registerCommand("Score LP3 Right",
                                new DoScorePositionCommand(
                                                elevatorSubsystem,
                                                coralManipulatorSubsystem,
                                                m_robotDrive,
                                                3, // Level 3 scoring position
                                                OIConstants.rightScoringOffset,
                                                OIConstants.scoringDistanceRight,
                                                CoralManipulatorConstants.levelThreePivotPosition));

        }

        

        /** Represents modes for different controls */
        enum ControlMode {
                /**
                 * Elevator and coral manipulator move but robot does not
                 * approach automatically after d-pad mapping
                 */
                Manual,
                /** Robot automatically scores / intakes coral after
                 * d-pad mapping
                 */
                SemiAuto;

                /**
                 * Returns if object is Manual
                 * @return boolean
                 */
                boolean manual() {
                        return this.equals(Manual);
                }

                /**
                 * Returns if object is SemiAuto
                 * @return boolean
                 */
                boolean semiAuto() {
                        return this.equals(SemiAuto);
                }
        }

        Double driverPOVRecency = null;
        POV driverLatestPOVButton = POV.None;

        Double operatorPOVRecency = null;
        POV operatorLatestPOVButton = POV.None;

        ControlMode activeMode = ControlMode.SemiAuto;

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
                        .onTrue(new InstantCommand(() -> {
                                elevatorSubsystem.turnOffAutoMode();
                        })).whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                if (activeMode == ControlMode.Manual) {
                                      elevatorSubsystem.setMotorSpeed(-c_operatorController.getLeftY());  
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
                                        coralManipulatorSubsystem.movePivotTo(
                                                coralManipulatorSubsystem.getPivotMotorPosition() + -0.025 * c_operatorController.getRightY()
                                        );
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
                                algaeManipulatorSubsystem.setIntakeSpeed(-1);
                        })).onFalse(new InstantCommand(() -> {
                                algaeManipulatorSubsystem.stopIntake();
                        }));

                // Right trigger -
                c_operatorController.rightTrigger(OIConstants.kTriggerThreshold)
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                if (!(c_operatorController.getLeftTriggerAxis() > OIConstants.kTriggerThreshold))
                                        coralManipulatorSubsystem.startIntakeMotor(-1 * c_operatorController.getRightTriggerAxis());
                                else 
                                        coralManipulatorSubsystem.stopIntakeMotor();
                        }))).onFalse(new InstantCommand(() -> {
                                coralManipulatorSubsystem.stopIntakeMotor();
                        }));

                // Left bumper - Coral manipulator wheels out
                c_operatorController.leftBumper()
                        .onTrue(new InstantCommand(() -> {
                                algaeManipulatorSubsystem.setIntakeSpeed(1);
                        })).onFalse(new InstantCommand(() -> {
                                algaeManipulatorSubsystem.stopIntake();
                        }));

                // Left trigger -
                c_operatorController.leftTrigger(OIConstants.kTriggerThreshold)
                        .whileTrue(new RepeatCommand(new InstantCommand(() -> {
                                if (!(c_operatorController.getRightTriggerAxis() > OIConstants.kTriggerThreshold))
                                        coralManipulatorSubsystem.startIntakeMotor(c_operatorController.getLeftTriggerAxis());
                                else 
                                        coralManipulatorSubsystem.stopIntakeMotor();
                        }))).onFalse(new InstantCommand(() -> {
                                coralManipulatorSubsystem.stopIntakeMotor();
                        }));

                // Y button - Toggle Coral Mode
                c_operatorController.y()
                        .onTrue(new InstantCommand(() -> {
                                elevatorSubsystem.moveToPosition(ElevatorConstants.highAlgaeClear);
                                coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.clearPivotHeight);
                        }));

                // X button - Algae Reef Clear Mode
                c_operatorController.x()
                        .onTrue(new InstantCommand(() -> {
                                elevatorSubsystem.moveToPosition(ElevatorConstants.lowAlgaeClear);
                                coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.clearPivotHeight);
                        }));

                // B button - Algae intake mode
                c_operatorController.b()
                        .onTrue(new InstantCommand(() -> {
                                algaeManipulatorSubsystem.setDesiredPivotHeight(AP.intaking);
                        }));

                // A button - 
                c_operatorController.a()
                        .onTrue(new TransitModeCommand(elevatorSubsystem, coralManipulatorSubsystem, algaeManipulatorSubsystem));

                // Right Stick button - Transit mode
                c_operatorController.rightStick()
                        .onTrue(new InstantCommand());

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
                                                if (activeMode.semiAuto()) {
                                                        new DoIntakeCoralFromStationCommand(
                                                                elevatorSubsystem, 
                                                                coralManipulatorSubsystem, 
                                                                m_robotDrive)
                                                        .schedule();
                                                } else if (activeMode.manual()) {
                                                        elevatorSubsystem.moveToPosition(ElevatorConstants.coralStationPosition);
                                                        coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.intakePivotPosition);
                                                }
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Up;
                        }));

                // Dpad Up-Right button -
                c_operatorController.povUpRight()
                        .onTrue(new InstantCommand());

                // Dpad Right button -
                c_operatorController.povRight()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press -
                                        // Coral mode: after down press: score L2 right
                                        // Coral mode: after up press: score L3 right
                                        if (operatorLatestPOVButton == POV.Down) {
                                                if (activeMode.semiAuto()) {
                                                        new DoScorePositionCommand(
                                                                elevatorSubsystem, 
                                                                coralManipulatorSubsystem, 
                                                                m_robotDrive,
                                                                2, 
                                                                OIConstants.rightScoringOffset, 
                                                                OIConstants.scoringDistanceRight, 
                                                                CoralManipulatorConstants.levelTwoPivotPosition)
                                                        .schedule();
                                                } else if (activeMode.manual()) {
                                                        elevatorSubsystem.moveToScoringPosition(SP.two);
                                                        coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.levelTwoPivotPosition);
                                                }
                                        }
                                        if (operatorLatestPOVButton == POV.Up) {
                                                if (activeMode.semiAuto()) {
                                                        new DoScorePositionCommand(
                                                                elevatorSubsystem, 
                                                                coralManipulatorSubsystem, 
                                                                m_robotDrive,
                                                                3, 
                                                                OIConstants.rightScoringOffset, 
                                                                OIConstants.scoringDistanceRight, 
                                                                CoralManipulatorConstants.levelThreePivotPosition)
                                                        .schedule();
                                                } else if (activeMode.manual()) {
                                                        elevatorSubsystem.moveToScoringPosition(SP.three);
                                                        coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.levelThreePivotPosition);
                                                }
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Right;
                        }));

                // Dpad Down-Right button -
                c_operatorController.povDownRight()
                        .onTrue(new InstantCommand());

                // Dpad Down button -
                c_operatorController.povDown()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press - Coral mode: after down press: score L1
                                        if (operatorLatestPOVButton == POV.Down) {
                                                if (activeMode.semiAuto()) {
                                                        new DoScorePositionCommand(
                                                                elevatorSubsystem, 
                                                                coralManipulatorSubsystem, 
                                                                m_robotDrive,
                                                                1, 
                                                                0.0, 
                                                                OIConstants.scoringDistanceRight, 
                                                                CoralManipulatorConstants.levelOnePivotPosition)
                                                        .schedule();
                                                } else if (activeMode.manual()) {
                                                        elevatorSubsystem.moveToScoringPosition(SP.one);
                                                        coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.levelOnePivotPosition);
                                                }
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Down;
                        }));

                // Dpad Down-Left button -
                c_operatorController.povDownLeft()
                        .onTrue(new InstantCommand());

                // Dpad Left button -
                c_operatorController.povLeft()
                        .onTrue(new InstantCommand(() -> {
                                if (operatorPOVRecency != null && 
                                        operatorPOVRecency + OIConstants.doublePressBuffer > Timer.getFPGATimestamp()) {
                                        // on Double Press -
                                        // Coral mode: after down press: score L2 left
                                        // Coral mode: after up press: score L3 left
                                        if (operatorLatestPOVButton == POV.Down) {
                                                if (activeMode.semiAuto()) {
                                                        new DoScorePositionCommand(
                                                                elevatorSubsystem, 
                                                                coralManipulatorSubsystem, 
                                                                m_robotDrive,
                                                                2, 
                                                                OIConstants.leftScoringOffset, 
                                                                OIConstants.scoringDistanceLeft, 
                                                                CoralManipulatorConstants.levelTwoPivotPosition)
                                                        .schedule();
                                                } else if (activeMode.manual()) {
                                                        elevatorSubsystem.moveToScoringPosition(SP.two);
                                                        coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.levelTwoPivotPosition);
                                                }
                                        }
                                        if (operatorLatestPOVButton == POV.Up) {
                                                if (activeMode.semiAuto()) {
                                                        new DoScorePositionCommand(
                                                                elevatorSubsystem, 
                                                                coralManipulatorSubsystem, 
                                                                m_robotDrive,
                                                                3, 
                                                                OIConstants.leftScoringOffset, 
                                                                OIConstants.scoringDistanceLeft, 
                                                                CoralManipulatorConstants.levelThreePivotPosition)
                                                        .schedule();
                                                } else if (activeMode.manual()) {
                                                        elevatorSubsystem.moveToScoringPosition(SP.three);
                                                        coralManipulatorSubsystem.movePivotTo(CoralManipulatorConstants.levelThreePivotPosition);
                                                }
                                        }
                                } else {
                                        // on Single Press
                                }

                                operatorPOVRecency = Timer.getFPGATimestamp();
                                operatorLatestPOVButton = POV.Left;
                        }));

                // Dpad Up-Left button -
                c_operatorController.povUpLeft()
                        .onTrue(new InstantCommand());

                SmartDashboard.putBoolean("Manual Mode", activeMode.manual());

                // Start Button button - Manual mode on 0.5 second hold
                c_operatorController.start()
                        .onTrue(new InstantCommand(() -> {
                                operatorStartButtonTimestamp = Timer.getFPGATimestamp();
                                
                                activeMode = (activeMode == ControlMode.SemiAuto) ? 
                                                ControlMode.Manual : ControlMode.SemiAuto;
                                                
                                        ControllerUtils.Rumble(c_operatorController.getHID(), 0.5);
                                        SmartDashboard.putBoolean("Manual Mode", activeMode.manual());

                        })).onFalse(new InstantCommand(() -> {
                                if (Timer.getFPGATimestamp() > operatorStartButtonTimestamp + 0.5) {

                                        
                                        operatorStartButtonTimestamp = Double.NEGATIVE_INFINITY;

                                }

                        }));

                // Back Button button - Cancel all actions
                c_operatorController.back()
                        .onTrue(new InstantCommand(() -> {
                                stopRobot();
                        }));

                /* * * * * * * * * * * * *\
                 *                       *
                 * DRIVER BUTTON MAPPING *
                 *                       *
                \* * * * * * * * * * * * */

                // Right bumper -
                c_driverController.rightBumper()
                        .onTrue(new InstantCommand());

                // Right trigger -
                c_driverController.rightTrigger(OIConstants.kTriggerThreshold)
                        .onTrue(new InstantCommand(() -> {
                                if (activeMode.manual())
                                climbingArmSubsystem.setMotorSpeeds(c_driverController.getRightTriggerAxis());
                        })).onFalse(new InstantCommand(() -> {
                                climbingArmSubsystem.stopMotors();
                        }));

                // Left bumper -
                c_driverController.leftBumper()
                        .onTrue(new InstantCommand());

                // Left trigger -
                c_driverController.leftTrigger(OIConstants.kTriggerThreshold)
                        .onTrue(new InstantCommand(() -> {
                                if (activeMode.manual())
                                climbingArmSubsystem.setMotorSpeeds(-c_driverController.getLeftTriggerAxis());
                        })).onFalse(new InstantCommand(() -> {
                                climbingArmSubsystem.stopMotors();
                        }));

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
                        .onTrue(new InstantCommand(() -> {
                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Up;
                        }));

                // Dpad Up-Right button -
                c_driverController.povUpRight()
                        .onTrue(new InstantCommand());

                // Dpad Right button -
                c_driverController.povRight()
                        .onTrue(new InstantCommand(() -> {
                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Right;
                        }));

                // Dpad Down-Right button -
                c_driverController.povDownRight()
                        .onTrue(new InstantCommand());

                // Dpad Down button -
                c_driverController.povDown()
                        .onTrue(new InstantCommand(() -> {
                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Down;
                        }));

                // Dpad Down-Left button -
                c_driverController.povDownLeft()
                        .onTrue(new InstantCommand());

                // Dpad Left button -
                c_driverController.povLeft()
                        .onTrue(new InstantCommand(() -> {
                                driverPOVRecency = Timer.getFPGATimestamp();
                                driverLatestPOVButton = POV.Left;
                        }));

                // Dpad Up-Left button -
                c_driverController.povUpLeft()
                        .onTrue(new InstantCommand());

                // Start Button button -
                c_driverController.start()
                        .onTrue(new InstantCommand(() -> {
                                m_robotDrive.zeroHeading();
                        }));

                // Back Button button -
                c_driverController.back()
                        .onTrue(new InstantCommand(() -> {
                                stopRobot();
                        }));

        }

        public void stopRobot() {
                CommandScheduler.getInstance().cancelAll();
                elevatorSubsystem.turnOffAutoMode();
                coralManipulatorSubsystem.stopIntakeMotor();
                coralManipulatorSubsystem.stopPivotMotor();
                climbingArmSubsystem.stopMotors();
                algaeManipulatorSubsystem.stopIntake();
                algaeManipulatorSubsystem.stopPivot();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
