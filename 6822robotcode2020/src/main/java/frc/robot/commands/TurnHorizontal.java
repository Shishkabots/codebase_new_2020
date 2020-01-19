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
    double P = 0.01; // tune this
    int cameraX = 320;
    int errorX = 0;
    int targetX;
    double voltage = 0;
    public TurnHorizontal(int diffX) {
        requires(Robot.m_drivetrain);
        errorX = diffX;
    }
    
    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
        
    }
    
    protected void execute() {
        while (errorX != 0) {
            voltage = P * errorX;
            Robot.m_drivetrain.moveWithCurve(0,voltage,true);
        }

        
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
