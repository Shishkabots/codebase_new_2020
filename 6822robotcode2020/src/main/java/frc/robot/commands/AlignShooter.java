
package frc.robot.commands;

import frc.robot.subsystems.Turret;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.vision.VisionThread;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.core.*;
import java.util.ArrayList;

public class AlignShooter extends CommandGroup {
    public Mat img;
    public int errorx, errory;

    public double P = 0.02; // tune this
    public double I = 0.0;
    public double D = 0.0003;

    public int cameraX = Robot.imgWidth;
    public int targetX;
    public double voltage = 0;

    public static double integral, previous_error, derivative = 0;
    public double dt = 0.02;
    public double ff = 0.13; // needs to be tuned
    public double maxVoltage = 0.40 + ff;

    public double threshold = 5; // pixel error required before stopping

    public Turret turret = Robot.m_turret;

    public int[] findCenter() {
        // [x,y]
        int[] centerCoor = { -1, -1 };

        Robot.pipeline.process(img);
        ArrayList<MatOfPoint> contours = Robot.pipeline.filterContoursOutput();
        if (Robot.pipeline.filterContoursOutput().size() == 1) {
            Rect boundingRect = Imgproc.boundingRect(contours.get(0));
            centerCoor[0] = (int) (boundingRect.x + (boundingRect.width / 2.0));
            centerCoor[1] = (int) (boundingRect.y);
            return centerCoor;
        }
        return centerCoor;
    }

    public AlignShooter(Mat img) {
        this.img = img;
    }

    protected void initialize() {

    }

    protected void execute() {
        int[] center = findCenter();
        errorx = center[0] - Robot.imgWidth;
        errory = center[1] - Robot.imgHeight;
        if (Math.abs(errorx) > 2) {
            // The condition if error = 0 is being checked in visionController, thats why you dont need it here
            // this turn vertical command is only being called when the condition ^ is false
            integral += (errorx * dt); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
            derivative = (errorx - previous_error) / dt;
            previous_error = errorx; //keep updating error to the most recently measured one
            double voltage = (P * errorx + I * integral + D * derivative);
            voltage += (errorx > 0 ? ff : -ff);
            if (Math.abs(voltage) >= maxVoltage) {
                voltage = Math.signum(voltage) * maxVoltage;
            }
            turret.rotate(voltage);
        }
    }

    protected boolean isFinished() {
        return (Math.abs(errorx) < threshold);
    }

    protected void end() {
        addSequential(new StopTurning());
        //addSequential(new Shoot(getDistMeter(), Robot.heightOuterPort));
    }

    protected void interrupted() {
    }
}
