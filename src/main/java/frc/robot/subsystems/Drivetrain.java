package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

public class Drivetrain extends Subsystem {

    private SwerveModule topLeft;
    private SwerveModule topRight;
    private SwerveModule bottomLeft;
	private SwerveModule bottomRight;

	public Drivetrain() {
		topLeft = new SwerveModule(RobotMap.TOP_LEFT_DRIVE_ID, RobotMap.TOP_LEFT_ANGLE_ID);
		topRight = new SwerveModule(RobotMap.TOP_RIGHT_DRIVE_ID, RobotMap.TOP_RIGHT_ANGLE_ID);
		bottomLeft = new SwerveModule(RobotMap.BOTTOM_LEFT_DRIVE_ID, RobotMap.BOTTOM_LEFT_ANGLE_ID);
		bottomRight = new SwerveModule(RobotMap.BOTTOM_RIGHT_DRIVE_ID, RobotMap.BOTTOM_RIGHT_ANGLE_ID);
	}

	public SwerveModule getTopLeft() {
		return topLeft;
	}

	public SwerveModule getTopRight() {
		return topRight;
	}

	public SwerveModule getBottomLeft() {
		return bottomLeft;
	}

	public SwerveModule getBottomRight() {
		return bottomRight;
	}

	public void talonInit() {
		
	}
	
	@Override
	protected void initDefaultCommand() {
		
	}
    
}