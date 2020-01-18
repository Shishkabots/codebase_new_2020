package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public static DifferentialDrive drivy;
    public static boolean reverse;

	public DriveTrain()
	{
        drivy = Robot.m_drive;
        reverse = true;
		//super("DriveTrain",0.2,0,0);
        //setAbsoluteTolerance(0.05);
        
    }

	
    public void initDefaultCommand() {
       //setDefaultCommand(new DriveTrainControl());
    }
    
    /**
     * regular move
     * @param left
     * @param right
     */
    public void move(double left, double right)
    {
    		drivy.tankDrive(left, right);
    }

    /**
     * Moves with angle
     * @param forward
     * @param angle
     * @param inPlaceTurn
     */
    public void moveWithCurve(double forward, double angle, boolean inPlaceTurn)
    {
    	drivy.curvatureDrive(forward, angle, inPlaceTurn);
    }
    public void arcadeDrive(double forward, double angle){
        drivy.arcadeDrive(forward, angle);
    }

	/*@Override
	protected double returnPIDInput() {
		return RobotMap.sonicSensor.getRangeInches();
	}
	@Override
	protected void usePIDOutput(double output) {
		drivy.tankDrive(-output, -output);
	}   */
}
