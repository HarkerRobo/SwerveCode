package frc.robot.util;

import harkerrobolib.wrappers.HSTalon;

public class SwerveModule
{
    private HSTalon angleMotor;
    private HSTalon driveMotor;

    public SwerveModule(int driveId, int angleId)
    {
        driveMotor = new HSTalon(driveId);
        angleMotor = new HSTalon(angleId);
    }        
    
    public HSTalon getAngleMotor()
    {
        return angleMotor;    
    }            
        
    public HSTalon getDriveMotor() {
        return driveMotor;
    } 
    
    public void talonInit() {
        
    }
}