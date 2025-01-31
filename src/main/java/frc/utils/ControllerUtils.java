// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

/** Util for Xbox controller */
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

}
