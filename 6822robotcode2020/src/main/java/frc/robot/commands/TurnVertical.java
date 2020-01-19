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
    double P = 0.08; // tune this
    int cameraY = 240;
    int errorY = 0;
    int targetY;
    double voltage = 0;
    public TurnVertical(int goalY) {
        requires(Robot.m_drivetrain);
        targetY= goalY;
    }
    
    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
        
    }
    
    protected void execute() {
        while (errorY != 0) {
            voltage = P * errorY;
            Robot.m_drivetrain.moveWithCurve(0,voltage,true);
        }
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
