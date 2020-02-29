package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.ctre.phoenix.motorcontrol.can.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
   
    public static TalonSRX lead;
    public static VictorSPX slave;

	public Shooter(TalonSRX a, VictorSPX b)
	{
        lead = a;
        slave = b;
        lead.setInverted(true);

        // allowing the two to spin separately
        //slave.follow(lead);
    }

    public void initDefaultCommand() {
        //setDefaultCommand(new DriveTrainControl());
        SmartDashboard.putNumber("Shooter Voltage", 0);
     }

    public void shoot(double voltageShoot) {
        SmartDashboard.putNumber("Shooter Voltage", voltageShoot);
        lead.set(ControlMode.PercentOutput, voltageShoot);
        slave.set(ControlMode.PercentOutput, voltageShoot);
    }

    // spin separately, if desired (only allows one at a time, however)
    public void shootLead(double voltageShoot) {
        SmartDashboard.putNumber("Lead Shooter Voltage", voltageShoot);
        lead.set(ControlMode.PercentOutput, voltageShoot);
        slave.set(ControlMode.PercentOutput, 0);
    }

    public void shootSlave(double voltageShoot) {
        SmartDashboard.putNumber("Slave Shooter Voltage", voltageShoot);
        slave.set(ControlMode.PercentOutput, voltageShoot);
        lead.set(ControlMode.PercentOutput, 0);
    }

    

}