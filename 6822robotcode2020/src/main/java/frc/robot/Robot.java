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

import edu.wpi.first.wpilibj.AnalogInput;

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

  public static WPI_VictorSPX turret;

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
  public static Turret m_turret;

  public static OI m_oi;
  public static Thread m_visionThread;
  public static CvSink cvSink;

  public static GripPipeline pipeline;
  public static final int imgWidth = 640;
  public static final int imgHeight = 480;

  public static final int kUltrasonicPort0 = 0;
  public static final int kUltrasonicPort1 = 1;
  public static final double kValueToInches = 1;
  public static final int minValue = 238;
  public static final double mvPer5mm = 0.004885;
  public static double theta = 0;
  public static Spark led;

  public static final AnalogInput m_ultrasonic0 = new AnalogInput(kUltrasonicPort0);
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static final double heightOuterPort = 2.4892; //units in meters
  private int cont;
  
  
  public int[] findCenter(MatOfPoint contour) {
    // [x,y]
    int[] centerCoor = { -1, -1 };
    Rect boundingRect = Imgproc.boundingRect(contour);
    centerCoor[0] = (int) (boundingRect.x + (boundingRect.width / 2.0));
    centerCoor[1] = (int) (boundingRect.y + (boundingRect.height/2));
    return centerCoor;
  }

  public MatOfPoint getLargestContour(ArrayList<MatOfPoint> contours)
  {
    int index = 0 ;
    int largestArea = 0;
    for(int i=0;i<contours.size();i++)
    {
      Rect boundingRect = Imgproc.boundingRect(contours.get(i));
      if(boundingRect.width * boundingRect.height > largestArea)
      {
        index = i;
        largestArea = boundingRect.width * boundingRect.height;
      }
    }
    return contours.get(index);
  }
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_ultrasonic0.setGlobalSampleRate(10000);
    SmartDashboard.putNumber("Ultrasonic Sensor 0", 5.0*m_ultrasonic0.getVoltage()/mvPer5mm);
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
        //m_oi.autoAlignButton.whenPressed(new AlignShooter(img));
        //new AlignShooter(img);
        Robot.pipeline.process(img);
        ArrayList<MatOfPoint> contours = Robot.pipeline.filterContoursOutput();
        if(contours.size()>0)
        {
          int center[] = findCenter(getLargestContour(contours));
          Imgproc.circle(img, new Point(center[0],center[1]),10,new Scalar(255,255,0),10);
        }
        Imgproc.circle(img, new Point(imgWidth/2,imgHeight/2),10,new Scalar(255,255,0),10);
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

    shoot1.configPeakCurrentLimit(60);
    shoot1.configContinuousCurrentLimit(28);

    drive1 = new WPI_TalonFX(9);
    drive2 = new WPI_TalonFX(10);
    slave1 = new WPI_TalonFX(11);
    slave2 = new WPI_TalonFX(12);

    turret = new WPI_VictorSPX(13);

    m_turret = new Turret(turret);

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
    SmartDashboard.putNumber("Ultrasonic Sensor 0", 5.0 * m_ultrasonic0.getAverageVoltage() / mvPer5mm);
  }

  @Override
  public void autonomousPeriodic() {
    double currentDistanceAuto = ((m_ultrasonic0.getValue())) * kValueToInches;
    SmartDashboard.putNumber("auto reading", currentDistanceAuto);
    //System.out.println("auto reading" + currentDistanceAuto);

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
    cont += 5;
    //double currentDistanceTeleop = (m_ultrasonic.getAverageVoltage()-minVoltage) *  kValueToInches;
    if(cont%200 == 0) {
      SmartDashboard.putNumber("Ultrasonic Sensor 0", 5.0 * m_ultrasonic0.getAverageVoltage() / mvPer5mm);
      double currentDistanceTeleop = 5.0 * (((m_ultrasonic0.getAverageVoltage() / mvPer5mm);
      SmartDashboard.putNumber("Teleop Distance", currentDistanceTeleop);
      //System.out.println("readings: " + currentDistanceTeleop);
    }
  }

  @Override
  public void testPeriodic() {
  }
}
