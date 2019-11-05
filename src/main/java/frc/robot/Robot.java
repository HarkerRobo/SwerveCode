/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        OI.initBindings();

        Drivetrain.getInstance();
    }

    /**
     * This function is called periodically once the robot has started up, regardless of mode.
     */
    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is run once each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters teleoperated mode.
     */
    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically when the robot is disabled.
     */
    @Override
    public void disabledPeriodic() {
        Drivetrain.getInstance().getTopLeft().getDriveMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getTopLeft().getAngleMotor().set(ControlMode.Disabled, 0);

        Drivetrain.getInstance().getTopRight().getDriveMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getTopRight().getAngleMotor().set(ControlMode.Disabled, 0);

        Drivetrain.getInstance().getBottomLeft().getDriveMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getBottomLeft().getAngleMotor().set(ControlMode.Disabled, 0);

        Drivetrain.getInstance().getBottomRight().getDriveMotor().set(ControlMode.Disabled, 0);
        Drivetrain.getInstance().getBottomRight().getAngleMotor().set(ControlMode.Disabled, 0);
    }
}
