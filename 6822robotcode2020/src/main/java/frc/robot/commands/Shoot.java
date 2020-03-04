package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.OI;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.Timer;

public class Shoot extends CommandGroup {
    double d;
    double u;
    AnalogInput beam2 = Robot.beam2;
    boolean ball2 = true;
    public Shoot() {
        //a AND b ARE THE HORIZONTAL AND VERTICAL TARGET DISTANCES RESPECTIVELY
        
    }
    //SET Robot.theta FROM Robot.java DEPENDING ON FINAL DESIGN
   double theta = Robot.theta;
    
    
    @Override
    protected void initialize() {
        
    }
    @Override
    protected void execute() {
        double num = -9.8*d*d/(2*(u-Math.tan(theta)*d)*(Math.cos(theta)*Math.cos(theta)));
        double v = Math.sqrt(num);
        //v IS THE VELOCITY WE NEED TO SHOOT AT
        //THIS IS A COMMAND GORUP SO THAT YOU ARE ABLE TO ADD A SHOOT COMMAND USING v AS AN INPUT
        if (ball2 && beam2.getValue() <= 100) {
            ball2 = false;
        }
        else if (!ball2 && beam2.getValue() > 100) {
            ball2 = true;
            Robot.ballcount--;
        }
    }

   
    @Override
    protected boolean isFinished() {

        return true;
    }

    
    @Override
    protected void end() {
    }

    
    @Override
    protected void interrupted() {
       
    }
}