// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

/** Add your docs here. */
public class ControllerUtils {
    public enum POV {
        Up(0),
        UpRight(45),
        Right(90),
        DownRight(135),
        Down(180),
        DownLeft(210),
        Left(270),
        UpLeft(315),
        None(-1);

        public final int value;

        POV(int value) {
            this.value = value;
        }
    }

}
