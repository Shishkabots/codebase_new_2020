package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

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
        requires(Robot.m_shooter);
        errorX = diffX;
    }

    Shooter shoot = Robot.m_shooter;
       
    protected void initialize() {
        shoot.shoot(0);
        
    }
    
    protected void execute() {
        // The condition if error = 0 is being checked in visionController, thats why you dont need it here
        // this turn vertical command is only being called when the condition ^ is false
        voltage = P * errorX;
        shoot.turnShooter(voltage);
        SmartDashboard.putString("Am I aligning?", "yes");     
          
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
        shoot.shoot(0);
    }

    protected void interrupted() {
    	shoot.shoot(0);
    }
}
