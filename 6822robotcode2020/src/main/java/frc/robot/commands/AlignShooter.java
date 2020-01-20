
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

public class AlignShooter extends CommandGroup{
    public int[] findCenter(Mat img) {
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
    public void align(Mat img)
    {
        int[] center = findCenter(img);
        int errorx = center[0] - Robot.imgWidth;
        int errory = center[1] - Robot.imgHeight;
        if(Math.abs(errorx)>2)
        {
            addSequential(new TurnHorizontal(errorx));
        }
        else if(Math.abs(errorx)>2)
        {
            addSequential(new TurnVertical(errory));
        }
        else
        {
            new StopTurning().start();
        }
    }

    public AlignShooter() {
    }
    
    protected void initialize() {
    }
    
    protected void execute() {        
    }

    protected boolean isFinished() {
        return false;
    }
    
    protected void end() {
    }

    protected void interrupted() {
    }
}
