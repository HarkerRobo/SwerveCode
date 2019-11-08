package frc.robot.commands;


/**
 * A useful robot mode during competitions.
 * @author Anirudh Kotamraju
 */
public class MemeMode extends TimedCommand {

    public static final int MEME_MODE_PERCENT_OUTPUT  = 0.5;

    public MemeMode(int time) {
        super(time);
    }

    
    public void initialize() {
        
    }


    public void execute() {    
        //Position angle motors while spinning.
        Drivetrain.getInstance().getTopRight().getAngleMotor().setAngle(135);
        Drivetrain.getInstance().getBottomLeft().getAngleMotor().setAngle(315);
        Drivetrain.getInstance().getTopLeft().getAngleMotor().setAngle(45);
        Drivetrain.getInstance().getBottomRight().getAngleMotor().setAngle(225);
        
        //Hopefully makes the robot spin.
        Drivetrain.getInstance().getTopRight().getDriveMotor().set(ControlMode.PercentOutput, MEME_MODE_PERCENT_OUTPUT);
        Drivetrain.getInstance().getBottomRight().getDriveMotor().set(ControlMode.PercentOutput, MEME_MODE_PERCENT_OUTPUT);
        Drivetrain.getInstance().getTopLeft().getDriveMotor().set(ControlMode.PercentOutput, MEME_MODE_PERCENT_OUTPUT);
        Drivetrain.getInstance().getBottomLeft().getDriveMotor().set(ControlMode.PercentOutput, MEME_MODE_PERCENT_OUTPUT);
    }

    public boolean interrupted() {
        end();
    }

    public boolean isFinished() {
        return isTimedOut();
    }
}