package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Turret extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
   
    public static TalonSRX turner;

    public Turret(TalonSRX b)
	{
        turner = b;   
    }

    public void initDefaultCommand() {
        //setDefaultCommand(new DriveTrainControl());
        SmartDashboard.putNumber("Turret Turning Voltage", 0);
        
    }

     public void rotate(double voltage) {
        SmartDashboard.putNumber("Turret Turning Voltage", voltage);
        turner.set(ControlMode.PercentOutput, voltage);
     }
}