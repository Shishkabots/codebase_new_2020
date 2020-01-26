package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Turret;

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

    Turret turret = Robot.m_turret;
       
    protected void initialize() {
        turret.rotate(0);
        
    }
    
    protected void execute() {
        // The condition if error = 0 is being checked in visionController, thats why you dont need it here
        // this turn vertical command is only being called when the condition ^ is false
        voltage = P * errorX;
        turret.rotate(voltage);
        SmartDashboard.putString("Am I aligning?", "yes");     
          
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
        turret.rotate(0);
    }

    protected void interrupted() {
    	turret.rotate(0);
    }
}
