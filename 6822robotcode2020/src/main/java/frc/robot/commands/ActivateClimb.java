package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class ActivateClimb extends Command {
    public ActivateClimb() {
        //requires(Robot.m_climber);
    }
    Climber climb = Robot.m_climber;
    int time;
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        climb.lift(0);
        time = 0;
    }
        
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        time += 100;
        climb.lift(0.3);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {

        return time >= 1900;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        climb.lift(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        climb.lift(0);
    }
}