package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnVertical extends Command {
    
    public TurnVertical(double tt) {
        requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
        
    }
    
    protected void execute() {
    }

    protected boolean isFinished() {
        return true;
    }
    
    protected void end() {
        Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}
