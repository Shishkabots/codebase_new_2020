package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class StopRobot extends Command {
    public StopRobot() {
       requires(Robot.m_drivetrain);
    }

            // Called just before this Command runs the first time
            
    @Override
    protected void initialize() {
        
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        new TeleOpCommands().start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	//Robot.m_hatch.setState("Off");
    }
}