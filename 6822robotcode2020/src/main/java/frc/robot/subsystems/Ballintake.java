package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Ballintake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
   
    public static VictorSPX lead;

	public Ballintake(VictorSPX a)
	{

        lead = a;
        
    }

    public void initDefaultCommand() {
        //setDefaultCommand(new DriveTrainControl());
        SmartDashboard.putNumber("Intake Voltage", 0);
        
     }

     public void spin(double voltage) {
        SmartDashboard.putNumber("Intake Voltage", voltage);
        lead.set(ControlMode.PercentOutput, voltage);
     }
}