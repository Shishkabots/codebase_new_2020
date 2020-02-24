package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 */
public class IntakeDropper extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public DoubleSolenoid Piston1 = Robot.leftSide;
    public DoubleSolenoid Piston2 = Robot.rightSide;


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void setState(String state){
        if(state == "Off"){
            Robot.leftSide.set(DoubleSolenoid.Value.kOff);
            Robot.rightSide.set(DoubleSolenoid.Value.kOff);
        }
        else if(state == "Close"){
            Robot.leftSide.set(DoubleSolenoid.Value.kForward);
            Robot.rightSide.set(DoubleSolenoid.Value.kForward);
        }
        else if(state == "Open"){
            Robot.leftSide.set(DoubleSolenoid.Value.kReverse);
            Robot.rightSide.set(DoubleSolenoid.Value.kReverse);
        }
    }
}