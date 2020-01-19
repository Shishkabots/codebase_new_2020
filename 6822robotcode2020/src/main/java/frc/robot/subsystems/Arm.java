package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Arm extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
   
    public static TalonSRX lead;
    public static VictorSPX slave;

	public Arm(TalonSRX a, VictorSPX b)
	{

        lead = a;
        slave = b;
        slave.follow(lead);
        
    }

    public void initDefaultCommand() {
        //setDefaultCommand(new DriveTrainControl());
        SmartDashboard.putNumber("Arm Voltage", 0);
        
     }

     public void lift(double voltage) {
        SmartDashboard.putNumber("Arm Voltage", voltage);
        lead.set(ControlMode.PercentOutput, voltage);
     }
}