/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.*;
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
import java.util.Arrays;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import java.lang.reflect.Array;
//import frc.robot.GripPipeline;
import java.util.ArrayList;
import java.util.Collections;
import java.io.*;

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

  public static WPI_VictorSPX arm1;
  public static WPI_VictorSPX arm2;

  public static WPI_TalonSRX shoot1;
  public static WPI_VictorSPX shoot2;

  public static WPI_TalonSRX turret;

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
    
//public static DoubleSolenoid leftSide = new DoubleSolenoid(3, 4);
//public static DoubleSolenoid rightSide = new DoubleSolenoid(4,5);

  public static OI m_oi;
  public static Thread m_visionThread;
  public static CvSink cvSink;
  public static CvSource outputStream;
  public static GripPipeline pipeline;
  public static final int imgWidth = 320;
  public static final int imgHeight = 240;

  //public static final int kUltrasonicPort0 = 0;
  //public static final int kUltrasonicPort1 = 1;
  public static final double kValueToInches = 1;
  public static final int minValue = 238;
  public static final double mvPer5mm = 0.004885;
  public static double theta = 0;
  public static Spark led;
  public static Mat img;

  public static UsbCamera camera;
  //public static final AnalogInput m_ultrasonic0 = new AnalogInput(kUltrasonicPort0);

  
  //public static IntakeDropper m_dropper = new IntakeDropper();

  public static boolean testing;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static final double heightOuterPort = 2.4892; // units in meters
  private int cont = 0;
  private double tempsum = 0;
  private double[] voltReading = new double[25];

  public final double FOVAngleWidth = Math.toRadians(58.5) / 2; // degrees
  public final double TcmWidth = 99.695;// width of vision target in cm
  public final int FOVpixelWidth = Robot.imgWidth;

  public final double FOVAngleHeight = Math.toRadians(45.6) / 2;
  public final double TcmHeight = 43.18; // heigh of vision target in cm
  public final int FOVpixelHeight = Robot.imgHeight;

  public final double Tratio = 0.475;// TcmHeight/TcmWidth;
  private TreeMap<Double, Double> distances = new TreeMap<Double, Double>();

  private static AnalogInput beam1;
  public  static AnalogInput beam2;
  public static boolean ball;
  public static int ballcount;

  public static Encoder encoder;


  public void fileTesting(MatOfPoint contour, String measuredDistance) throws IOException {
    FileWriter fw = new FileWriter("/home/lvuser/distanceData/output.txt");    
    fw.write(measuredDistance); 
    //System.out.println(measuredDistance);
    SmartDashboard.putString("measured distance", measuredDistance);
    fw.close();    
  }
  @Override
  public void robotInit() {
    encoder = new Encoder(0, 1);
    encoder.reset();
    //leftSide.set(DoubleSolenoid.Value.kForward);
    //rightSide.set(DoubleSolenoid.Value.kForward);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    AnalogInput.setGlobalSampleRate(100.0);
    //SmartDashboard.putNumber("Ultrasonic Sensor 0", 5.0 * m_ultrasonic0.getVoltage() / mvPer5mm);
    pipeline = new GripPipeline();

    m_oi = new OI();

    m_visionThread = new Thread(() -> {
      camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(imgWidth, imgHeight);
      cvSink = CameraServer.getInstance().getVideo();
      outputStream = CameraServer.getInstance().putVideo("Rectangle", imgWidth, imgHeight);
      img = new Mat();
      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(img) == 0) {
          outputStream.notifyError(cvSink.getError());
          continue;
        }
        pipeline.process(img);
        ArrayList<MatOfPoint> contours  = pipeline.filterContoursOutput();
        int index = 0;
        int largestArea = 0;
        for (int i = 0; i < contours.size(); i++) {
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            if (boundingRect.width * boundingRect.height > largestArea) {
                index = i;
                largestArea = boundingRect.width * boundingRect.height;
            }
        }
        Imgproc.drawContours(img, contours,index, new Scalar(0,0,255),5);
        outputStream.putFrame(img);
      }
      if(Thread.interrupted())
      {
        System.out.println("Vision Crashed");
      }
      
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    climb1 = new WPI_VictorSPX(9); // done
    climb2 = new WPI_VictorSPX(10); // done

    storage = new WPI_VictorSPX(12); // done
    intake = new WPI_VictorSPX(8); // done

    arm1 = new WPI_VictorSPX(13); // done
    arm1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    arm1.setSelectedSensorPosition(0);
    arm2 = new WPI_VictorSPX(11); // done
    arm1.setSelectedSensorPosition(0);

    shoot1 = new WPI_TalonSRX(6); //done
    shoot2 = new WPI_VictorSPX(7); // done

    shoot1.configPeakCurrentLimit(60);
    shoot1.configContinuousCurrentLimit(28);
    shoot1.setNeutralMode(NeutralMode.Coast);
    shoot2.setNeutralMode(NeutralMode.Coast);

    drive1 = new WPI_TalonFX(1); //done
    slave1 = new WPI_TalonFX(2); //done
    drive2 = new WPI_TalonFX(3); // done
    slave2 = new WPI_TalonFX(4); //done

    turret = new WPI_TalonSRX(5); // not done yet
    m_turret = new Turret(turret);

    m_drive = new DifferentialDrive(drive1, drive2);
    SmartDashboard.putData(m_drive);
    m_drivetrain = new DriveTrain();

    m_shooter = new Shooter(shoot1, shoot2);

    beam1 = new AnalogInput(3);
    beam2 = new AnalogInput(4);
    ball = true;
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    //SmartDashboard.putNumber("Ultrasonic Sensor 0", 5.0 * m_ultrasonic0.getAverageVoltage() / mvPer5mm);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }*/
    arm1.setSelectedSensorPosition(0);
    new TeleOpCommands().start();
    
    //new VisionProcess().start();
  }

  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
    case kCustomAuto:
      break;
    case kDefaultAuto:
    default:
      break;
    }
  }

  @Override
  public void teleopPeriodic() {
    cont++; 
    //System.out.println("teleop running");
    Scheduler.getInstance().run();
    
    /*System.out.println(arm1.getSelectedSensorPosition(0));
    if(Math.abs(arm1.getSelectedSensorPosition(0))<40000)
    {
      arm1.set(0.75);
    }
    else if(Math.abs(arm1.getSelectedSensorPosition(0))>41000)
    {
      arm1.set(0.1);
    }
    else
    {
      arm1.set(0);
    }*/

    //turret.set(-0.075);
    //shoot1.set(0.5); // 90k ticks, around 1320 rpm
    //shoot2.set(-0.5); // 40k ticks, around 590 rpm

    //shoot1.set(1); // 180k ticks, around 2640 rpm
    //shoot2.set(-1); // 78k ticks, around 1140 rpm
    
    //shoot1.set(1);
    //shoot2.set(1);
    //System.out.println(encoder.getRate()); 


    /*if(cont>150)
    {
      arm1.set(0.1);
      System.out.println(arm1.getSelectedSensorPosition(0));
    }
    else
    {
      arm1.set(-0.1);
      System.out.println(arm1.getSelectedSensorPosition(0));
    }*/
    if(ball && beam1.getValue() <= 100) {
      ball = false;
    }else if(!ball && beam1.getValue() > 100) {
      ball = true;
      ballcount++;
    }
  }

  @Override
  public void testPeriodic() {
  }
}
