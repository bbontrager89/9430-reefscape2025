// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Elastic.ElasticNotification;

/** Where all SmartDashbard vales will be edited */
public class SmartDashboardUtils extends SubsystemBase{

    private DriveSubsystem driveSubsystem;

    private PathPlannerPath spinPath;
    private PathPlannerPath driveForwarPath;
    private PathPlannerPath mForward;
    private PathPlannerPath mBackward;

    public SendableChooser<PathPlannerPath> pathChooser;

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

            pathChooser = new SendableChooser<PathPlannerPath>();

            spinPath = PathPlannerPath.fromPathFile("Spin");
            driveForwarPath = PathPlannerPath.fromPathFile("Drive Forward " + ((DriverStation.getAlliance().get() == Alliance.Red)? "Red": "Blue"));
            mForward = PathPlannerPath.fromPathFile("X Meters Forward");
            mBackward = PathPlannerPath.fromPathFile("X Meters Backwards");
            

            pathChooser.setDefaultOption("Go Forward " + ((DriverStation.getAlliance().get() == Alliance.Red)? "Red": "Blue"), driveForwarPath);
            pathChooser.addOption("Spin", spinPath);
            pathChooser.addOption("X Meters Forward", mForward);
            pathChooser.addOption("X Meters Backward", mBackward);



            SmartDashboard.putData("Path Chooser", pathChooser);



            SmartDashboard.putBoolean("Zero Heading", false);


           

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
            
            if(SmartDashboard.getBoolean("Zero Heading", false)) {
                SmartDashboard.putBoolean("Zero Heading", false);
                driveSubsystem.zeroHeading();
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void sendElasticNotification(ElasticNotification notification){
        Elastic.sendAlert(notification);
    }

}
