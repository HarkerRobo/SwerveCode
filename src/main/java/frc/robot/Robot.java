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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.util.Conversions;
import harkerrobolib.util.MathUtil;

/**
 * Send it? You don't know what it means? Well, let me tell you a little story. Think of the 'send' as the package. 
 * We're sending that package. Where? Doesn't matter. It's about the send, not the destination. Now, let's begin. 
 * Let's say you order a package off of Amazon™, but it never makes it past the website. That right there? That's 
 * a no send. No sender. Not even sent. Now you apply it to the Cheesy Poofs. Let's say we start a match, and the 
 * FMS data is sent out. But, the robot doesn't move. The Poofs don't pass the auto line, and they don't collect 
 * their five points. Bam. No send. Not a single aspect of the autonomous was sent. Anywhere.
 *
 * Half send? Well, let's say you order that package off of Amazon™. This time, it makes it past the website and 
 * your package leaves the warehouse; but, it's otherwise lost, stolen, or destroyed on its way to your house. 
 * Half send. That's a half sender. It probably got halfway there, but wasn't fully sent. Let's look back at the 
 * Poofs' auton. Now, the match starts, and Lockdown© moves. But, alas, its elevator hits the scale and the robot 
 * tips over, or only two of the four cubes successfully get placed. Bam, half send. That's a half sender. Half of 
 * those power cubes were sent.
 * 
 * Fully send, though? Well, let's, again, say you order that same package off of Amazon™. This time around, 
 * the package arrives all in one piece, no damage. You pick it up, sign for it, and use its contents. Now that, 
 * my friend, is a full send. Back to the auton though. Now the Poofs pickup those four power cubes and place all 
 * four of them on the scale. They now own that scale. Bam, full send. Those four cubes? Fully sent. You could say 
 * that those first fifteen seconds were a full send.
 * 
 * But, you go 53-0? Now, that's not a full send, that's a next level of send. That, my good friend, is a FULL 53ND.
 * This is the art of the send, and let me ask you: are you silly?
 */
public class Robot extends TimedRobot {

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        OI.getInstance().initBindings();
        Drivetrain.getInstance();
    }

    /**
     * This function is called periodically once the robot has started up, regardless of mode.
     */
    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();

        SmartDashboard.putNumber("BL Actual Angle", Drivetrain.getInstance().getBackLeft().getAngleDegrees());
        SmartDashboard.putNumber("TL Desired Angle", Drivetrain.getInstance().getTopLeft().getAngleDegrees());
        SmartDashboard.putNumber("TL Angle Error", MathUtil.constrain(Drivetrain.getInstance().getTopLeft().getAngleMotor().getClosedLoopError(), -100, 100));
        SmartDashboard.putNumber("TL Percent Output", Drivetrain.getInstance().getTopLeft().getAngleMotor().getOutputCurrent());
        // SmartDashboard.putNumber("TL RiseToFall", Drivetrain.getInstance().getTopLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("TR RiseToFall", Drivetrain.getInstance().getTopRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("BL RiseToFall", Drivetrain.getInstance().getBackLeft().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
        // SmartDashboard.putNumber("BR RiseToFall", Drivetrain.getInstance().getBackRight().getAngleMotor().getSensorCollection().getPulseWidthRiseToFallUs());
    
        SmartDashboard.putNumber("Pigeon Heading", Drivetrain.getInstance().getPigeon().getFusedHeading());

        SmartDashboard.putBoolean("Is field senstitive", Drivetrain.getInstance().isFieldSensitive());

        SmartDashboard.putNumber("TL Drive Pos", Drivetrain.getInstance().getTopLeft().getDriveMotor().getSelectedSensorPosition());
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
     * This function is called once each time the robot enters test mode.
     */
    @Override
    public void testInit() {
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
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.set(ControlMode.Disabled, 0));
        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.set(ControlMode.Disabled, 0));

        Drivetrain.getInstance().applyToAllDrive((talon) -> talon.clearMotionProfileTrajectories());
        Drivetrain.getInstance().applyToAllAngle((talon) -> talon.clearMotionProfileTrajectories());
    }
}
