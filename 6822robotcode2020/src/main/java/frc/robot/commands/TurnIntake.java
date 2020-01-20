package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Ballintake;
import frc.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class TurnIntake extends Command {
    public TurnIntake() {
        requires(Robot.m_intake);
    }
    Ballintake intake = Robot.m_intake;
    
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        intake.spin(0);
    }
        
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double speedCoef = 0.7;
        double lTrigger = Robot.m_oi.controllerTwo.getRawAxis(2);
        double rTrigger = Robot.m_oi.controllerTwo.getRawAxis(3);
        SmartDashboard.putNumber("ltrig", lTrigger);
        SmartDashboard.putNumber("rtrig", rTrigger);
        //voltage at which motor controlling intake should be spun at
        intake.spin((rTrigger - lTrigger) * speedCoef);    
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        intake.spin(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        intake.spin(0);
    }
}