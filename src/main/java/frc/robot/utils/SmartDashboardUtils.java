// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Elastic.ElasticNotification;

/** Where all SmartDashbard vales will be edited */
public class SmartDashboardUtils extends SubsystemBase{

    private DriveSubsystem driveSubsystem;

    /**
     * Constructor for SmartDashboardUtils
     * 
     * @param robotContainer
     */
    public SmartDashboardUtils(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        dashboardInit();
    }

    /**
     * Initializes the values for the SmartDashboard and gets all required instances
     */
    public void dashboardInit() {
        try {

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Updates the values on the SmartDashboard
     */
    @Override
    public void periodic() {
       try {

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void sendElasticNotification(ElasticNotification notification){
        Elastic.sendAlert(notification);
    }
}
