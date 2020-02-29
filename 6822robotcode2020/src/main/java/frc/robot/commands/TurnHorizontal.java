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
/*
public class TurnHorizontal extends Command {
    double P = 0.02; // tune this
    double I = 0.0;
    double D = 0.0003;

    int cameraX = 320;
    int errorX = 0;
    int targetX;
    double voltage = 0;

    double integral, previous_error, error, derivative = 0;
    double dt = 0.02;
    double ff = 0.13;
    double maxVoltage = 0.40 + ff;

    
    Turret turret = Robot.m_turret;

    public TurnHorizontal(int diffX) {
        requires(Robot.m_drivetrain); // necessary to make sure DT isn't moved during align?
        requires(Robot.m_shooter);
        errorX = diffX;
    }
       
    protected void initialize() {
        turret.rotate(0);
        
    }
    
    protected void execute() {
        // The condition if error = 0 is being checked in visionController, thats why you dont need it here
        // this turn vertical command is only being called when the condition ^ is false
        integral += (errorX * dt); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (errorX - previous_error) / dt;
        previous_error = errorX; //keep updating error to the most recently measured one
        double voltage = (P * errorX + I * this.integral + D * derivative);
        voltage += (error > 0 ? ff : -ff);
        if(Math.abs(voltage) >= maxVoltage){
            voltage = Math.signum(voltage) * maxVoltage;
            turret.rotate(voltage);
        }
        //SmartDashboard.putString("Am I aligning?", "yes");     
          
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
*/
// this is no longer necessary; we moved the PID into alignshooter.java