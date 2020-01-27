
package frc.robot.commands;

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

    public double getDistMeter() {
        return 5.0 * ((Robot.m_ultrasonic0.getVoltage() + Robot.m_ultrasonic1.getVoltage()) / 2)  / Robot.mvPer5mm / 1000;
    }

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
            addSequential(new TurnHorizontal(errorx));
        }
    }

    protected boolean isFinished() {
        return (Math.abs(errorx) < 2);
    }

    protected void end() {
        addSequential(new StopTurning());
        addSequential(new Shoot(getDistMeter(), Robot.heightOuterPort));
    }

    protected void interrupted() {
    }
}
