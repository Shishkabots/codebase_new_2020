package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnHorizontal extends Command {
    
    int t;
    public TurnHorizontal(int tt) {
        requires(Robot.m_drivetrain);
        t = tt;
    }
    
    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
        
    }
    
    protected void execute() {
        Robot.m_drivetrain.moveWithCurve(0,t,true);
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
        Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	Robot.m_drivetrain.move(0, 0);
    }
}
