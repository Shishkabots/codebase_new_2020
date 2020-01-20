/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.core.*;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
//import frc.robot.GripPipeline;
import java.util.ArrayList;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SPI;
//import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {

  public static WPI_VictorSPX climb1;
  public static WPI_VictorSPX climb2;

  public static WPI_VictorSPX storage;
  public static WPI_VictorSPX intake;

  public static WPI_TalonSRX arm1;
  public static WPI_VictorSPX arm2;

  public static WPI_TalonSRX shoot1;
  public static WPI_VictorSPX shoot2;

  public static WPI_TalonFX drive1;
  public static WPI_TalonFX drive2;
  public static WPI_TalonFX slave1;
  public static WPI_TalonFX slave2;

  public static DifferentialDrive m_drive;

  public static DriveTrain m_drivetrain;
  public static Arm m_arm;
  public static Ballintake m_intake;
  public static Climber m_climber;
  public static Shooter m_shooter;
  public static StorageFeed m_storage;

  public static OI m_oi;
  public static Thread m_visionThread;
  public static CvSink cvSink;

  public static GripPipeline pipeline;
  public static final int imgWidth = 640;
  public static final int imgHeight = 480;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    pipeline = new GripPipeline();

    m_oi = new OI();

    m_visionThread = new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
      Mat img = new Mat();
      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(img) == 0) {
          outputStream.notifyError(cvSink.getError());
          continue;
        }

        m_oi.autoAlignButton.whenPressed(new AlignShooter(img));
        int[] center = findCenter(img);
        Imgproc.circle(img, new Point(center[0],center[1]),2,new Scalar(255,0,0));
        Imgproc.circle(img, new Point(imgWidth/2,imgHeight/2),2,new Scalar(255,0,0));
        outputStream.putFrame(img);
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    climb1 = new WPI_VictorSPX(1);
    climb2 = new WPI_VictorSPX(2);

    storage = new WPI_VictorSPX(3);
    intake = new WPI_VictorSPX(4);

    arm1 = new WPI_TalonSRX(5);
    arm2 = new WPI_VictorSPX(6);

    shoot1 = new WPI_TalonSRX(7);
    shoot2 = new WPI_VictorSPX(8);

    drive1 = new WPI_TalonFX(9);
    drive2 = new WPI_TalonFX(10);
    slave1 = new WPI_TalonFX(11);
    slave2 = new WPI_TalonFX(12);

    m_drive = new DifferentialDrive(drive1, drive2);
    SmartDashboard.putData(m_drive);
    m_drivetrain = new DriveTrain(m_drive);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testPeriodic() {
  }
}
