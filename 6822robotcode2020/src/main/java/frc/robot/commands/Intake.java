package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends Command {
    //private int timecounter = 0;
    //private final int endtime = 100;
    
    public Intake() {
       //timecounter = 0;
    }

            // Called just before this Command runs the first time
            
    @Override
    protected void initialize() {
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        if(Robot.ext) {
            Robot.solenoid.set(DoubleSolenoid.Value.kForward);
            Robot.ext = false;
        }else {
            Robot.solenoid.set(DoubleSolenoid.Value.kReverse);
        }
        if(Robot.testing){
            SmartDashboard.putString("pogyes?: ", "BEGIN");
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	Robot.m_dropper.setState("Off");
    }
}