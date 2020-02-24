package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.AnalogGyro;

//import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class DriveTrainControl extends Command {

    //m_drivetrain is a drivetrain subsystem btw

    public DriveTrainControl() {
        requires(Robot.m_drivetrain);
    }
    /*Encoder e1 = Robot.e1;
    Encoder e2 = Robot.e2;
    AHRS gyro = Robot.gyro;*/
    double maxSpeed = 0;


    protected void initialize() {
        /*if(Robot.testing){
            SmartDashboard.putNumber("Interrupt: ", 0);
        }*/
        Robot.m_drivetrain.move(0, 0);
    }

    protected void execute() {
        double turboMultiplier = Robot.m_oi.boost.get() ? 2 : 1;
        //factor to multiply the speed by when we are on turbo mode
        double turnCoef = 0.5;
        //FF = Feedforward: A known value supplied to the output as a guesstimate so the PID only has to make minor corrections eventually changes
        double turnFF = 0.12;
        double forwardCoef = 0.65;
        double lTrigger = Robot.m_oi.controllerOne.getRawAxis(2);
        double rTrigger = Robot.m_oi.controllerOne.getRawAxis(3);
        double turnAxis = Robot.m_oi.controllerOne.getRawAxis(4);

        // -0.05 threshold is so turn direction is regular if you have no forward or are going forward
        // flips only when going backward (maybe make deadband less) so you go right if you are pushing the
        // joystick right, regardless of going reverse/forward
        // set this to 1 always if you want to restore "car" turning
        double turnDirection = (rTrigger - lTrigger > -0.05 ? 1 : -1);

        // if not under deadband (absolute value 5% voltage), don't add ff. If above deadband (either direction)
        // then apply sign (positive or negative) to the FF.
        turnFF = turnFF * (Math.abs(turnAxis) > 0.05 ? (turnAxis > 0 ? 1 : -1) : 0);

        //3 is right trigger, 2 is left trigger, 0 is x axis of left stick, unsure of math
    	Robot.m_drivetrain.moveWithCurve(
            (Robot.m_drivetrain.reverse ? -1 : 1) * (rTrigger - lTrigger) * forwardCoef * turboMultiplier,
            ((turnAxis * turnCoef * turboMultiplier) + turnFF) * turnDirection, 
            true
        );
        System.out.println(lTrigger+" "+rTrigger+" "+turnAxis);
        
        
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
    	Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
        //Robot.m_drivetrain.move(0, 0);
        /*if(Robot.testing){
            SmartDashboard.putNumber("Interrupt: ", 1);
        }*/
    }
}