
package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Spark;

public class Flash extends Command {
    private int ledCounter = 0;

    public Flash() {
       ledCounter = 0;
    }
    Spark ledd = Robot.led;
            // Called just before this Command runs the first time
            
    @Override
    protected void initialize() {
        ledd.set(0.45);
        ledCounter = 0;
    }
        
    // Called repeatedly when this Command is scheduled to run
    
    @Override
    protected void execute() {
        ledd.set(0.43);
        ledCounter++;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if(ledCounter > 40){ 
            return true; //after the loop runs 40 times and ledCounter = 40, then go to end() and reset counter
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        ledd.set(0.45);
        ledCounter = 0;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	
    }
}

