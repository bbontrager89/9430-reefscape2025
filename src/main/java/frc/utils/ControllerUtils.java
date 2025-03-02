// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Util for Xbox controllers */
public class ControllerUtils {
    /**Represents the D-pad on an Xbox controller */
    public enum POV {
        /** Up button */
        Up(0),
        /** Up-Right button */
        UpRight(45),
        /** Right button */
        Right(90),
        /** Down-Right button */
        DownRight(135),
        /** Down button */
        Down(180),
        /** Down-Left button */
        DownLeft(210),
        /** Left button */
        Left(270),
        /** Up-Left button */
        UpLeft(315),
        /** No button pressed */
        None(-1);

        /** The angle of the button */
        public final int value;

        POV(int value) {
            this.value = value;
        }
    }
    /**Represents the Axis's on an Xbox controller */
    public enum AXIS {
        /** Up button */
        LeftVertical(1),
        /** Up-Right button */
        LeftHorizontal(0),
        /** Right button */
        RightVertical(5),
        /** Down-Right button */
        RightHorizontal(4),
        /** Down button */
        LeftTrigger(2),
        /** Down-Left button */
        RightTrigger(3);

        /** The angle of the button */
        public final int value;

        AXIS(int value) {
            this.value = value;
        }
    }

    public static void Rumble(XboxController controller) {
        new SequentialCommandGroup(
            new InstantCommand(() -> {
             controller.setRumble(RumbleType.kBothRumble, 1);
            }), 
            new WaitCommand(1), 
            new InstantCommand(() -> {
                controller.setRumble(RumbleType.kBothRumble, 0);
            })).handleInterrupt(() -> {controller.setRumble(RumbleType.kBothRumble, 0);}).schedule();
    }

    public static void Rumble(XboxController controller, double time) {
        new SequentialCommandGroup(
            new InstantCommand(() -> {
             controller.setRumble(RumbleType.kBothRumble, 1);
            }), 
            new WaitCommand(time), 
            new InstantCommand(() -> {
                controller.setRumble(RumbleType.kBothRumble, 0);
            })).handleInterrupt(() -> {controller.setRumble(RumbleType.kBothRumble, 0);}).schedule();
    }

    public static void Rumble(XboxController controller, double time, double intencity) {
        new SequentialCommandGroup(
            new InstantCommand(() -> {
             controller.setRumble(RumbleType.kBothRumble, intencity);
            }), 
            new WaitCommand(time), 
            new InstantCommand(() -> {
                controller.setRumble(RumbleType.kBothRumble, 0);
            })).handleInterrupt(() -> {controller.setRumble(RumbleType.kBothRumble, 0);}).schedule();
    }

}
