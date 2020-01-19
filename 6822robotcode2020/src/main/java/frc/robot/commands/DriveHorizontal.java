package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.VisionController;
import frc.robot.GripPipeline;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.*;

import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveHorizontal extends Command {
    CvSink sink;
    public AHRS gyro = Robot.gyro; // angles are in degrees
    public double thetaToTurn; // t = target X-Coord
    private double centerX;
    // double P = 0.027;
    // double I = 0.016;
    // double D = 0.0001;
    double P = 0.01;
    double I = 0.0;
    double D = P/20;//0.0003;
    double integral, previous_error, error, derivative = 0;
    double dt = 0.02;
    double completionThreshold = 2.5; // also in degrees
    double ff = 0.13; // 0.14 < ff < 0.18 on hd meeting room carpet (this is ff to overcome kinetic, not static friction)
    
    double maxVoltage = 0.40 + ff; //max voltage we will use after accounting for feedforward things- friction etc.

    int itersUnderThreshold = 0;
    int itersComplete = 20;

    //m_drivetrain is a drivetrain subsystem btw
    public DriveHorizontal(int tt) {
        thetaToTurn = tt;
        requires(Robot.m_drivetrain);
    }
    
    protected void initialize() {
        Robot.m_drivetrain.move(0, 0);
        gyro.zeroYaw();
    }
    
    protected void execute() {
        //sink = VisionController.cvSink; 
        error = thetaToTurn - gyro.getAngle(); // Error = Target - Actual; 
        integral += (error * dt); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - previous_error) / dt;
        previous_error = error; //keep updating error to the most recently measured one
        double voltage = (P * error + I * this.integral + D * derivative);
        voltage += (error > 0 ? ff : -ff);
        if(Math.abs(voltage) >= maxVoltage){
            voltage = Math.signum(voltage) * maxVoltage;
        }
        Robot.m_drivetrain.moveWithCurve(0, voltage, true);
        SmartDashboard.putNumber("Gyro Voltage percentage: ", voltage);
        SmartDashboard.putNumber("Gyro Output Angle: ", gyro.getAngle());
        SmartDashboard.putNumber("Gyro Target Angle: ", t);
        SmartDashboard.putNumber("Gyro Integral: ", integral);
        SmartDashboard.putNumber("Gyro Angle Error: ", error);
        SmartDashboard.putNumber("Gyro Derivative: ", derivative);

        if(Math.abs(error) <= completionThreshold){
            itersUnderThreshold++;
        }
        else{
            itersUnderThreshold = 0;
        }
    }

    protected boolean isFinished() {
        return itersUnderThreshold >= itersComplete;
    }
    
    protected void end() {
        Robot.m_drivetrain.move(0, 0);
    }

    protected void interrupted() {
    	//Robot.m_drivetrain.move(0, 0);
    }
}