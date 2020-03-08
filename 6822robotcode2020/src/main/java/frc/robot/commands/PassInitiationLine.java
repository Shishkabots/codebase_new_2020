package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.AnalogGyro;
//import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class PassInitiationLine extends Command {

    //m_drivetrain is a drivetrain subsystem btw

    public PassInitiationLine() {
        requires(Robot.m_drivetrain);
    }

    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
    }

    protected void execute() {
        Robot.m_drivetrain.arcadeDrive(0.2, 0);
    }

    protected boolean isFinished() {
        return Math.abs(Robot.drive1.getSelectedSensorPosition())>10000 && Math.abs(Robot.drive2.getSelectedSensorPosition())>10000;
    }
    
    protected void end() {
    	Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    }
}