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
    
//public static DoubleSolenoid leftSide = new DoubleSolenoid(3, 4);
//public static DoubleSolenoid rightSide = new DoubleSolenoid(4,5);

  public static OI m_oi;
  public static Thread m_visionThread;
  public static CvSink cvSink;

  public static GripPipeline pipeline;
  public static final int imgWidth = 640;
  public static final int imgHeight = 480;

  //public static final int kUltrasonicPort0 = 0;
  //public static final int kUltrasonicPort1 = 1;
  public static final double kValueToInches = 1;
  public static final int minValue = 238;
  public static final double mvPer5mm = 0.004885;
  public static double theta = 0;
  public static Spark led;
  public static Mat img;

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

  public int[] findCenter(MatOfPoint contour) {
    // [x,y]
    int[] centerCoor = { -1, -1 };
    Rect boundingRect = Imgproc.boundingRect(contour);
    centerCoor[0] = (int) (boundingRect.x + (boundingRect.width / 2.0));
    centerCoor[1] = (int) (boundingRect.y + (boundingRect.height / 2));
    return centerCoor;
  }

  public MatOfPoint getLargestContour(ArrayList<MatOfPoint> contours) {
    int index = 0;
    int largestArea = 0;
    for (int i = 0; i < contours.size(); i++) {
      Rect boundingRect = Imgproc.boundingRect(contours.get(i));
      if (boundingRect.width * boundingRect.height > largestArea) {
        index = i;
        largestArea = boundingRect.width * boundingRect.height;
      }
    }
    return contours.get(index);
  }

  public double visionDistanceWidth(MatOfPoint contour) {
    double TPixelsWidth = Imgproc.boundingRect(contour).width;
    double TPixelsHeight = Imgproc.boundingRect(contour).height;
    double TPixelRatio = 1.0 * TPixelsHeight / TPixelsWidth;
    // System.out.println("/n"+TPixelRatio+" "+Tratio);
    double factor = 0.375 * (TPixelRatio / Tratio - 1) + 1;
    TPixelsWidth = TPixelsWidth * factor;
    double distX = (TcmWidth * FOVpixelWidth) / (2 * TPixelsWidth * Math.tan(FOVAngleWidth));
    return (1.20 * distX) + 5;
  }

  public double visionDistanceHeight(MatOfPoint contour) {
    double TPixelsHeight = Imgproc.boundingRect(contour).height;
    double distY = (TcmHeight * FOVpixelHeight) / (2 * TPixelsHeight * Math.tan(FOVAngleHeight));
    return (1.375 * distY) + 5;
  }

  private double visionError(double measuredDistance)
  {
    return -(0.128*measuredDistance+0.000273*(measuredDistance*measuredDistance))/1.5;
  }
  public double averageVisionDistance(MatOfPoint contour) {
    double measuredDistance =  (visionDistanceHeight(contour) + visionDistanceWidth(contour)) / 2.0;
    return measuredDistance - visionError(measuredDistance);
  }

  public void initializeDistanceTable() {
    distances.put(0.577, 12.780000610351562);
    distances.put(0.578, 12.630000152587892);
    distances.put(0.579, 12.460000305175782);
    distances.put(0.58, 12.300000305175782);
    distances.put(0.581, 12.140000610351564);
    distances.put(0.582, 11.990001220703125);
    distances.put(0.583, 11.85000244140625);
    distances.put(0.584, 11.720001220703125);
    distances.put(0.585, 11.5800048828125);
    distances.put(0.586, 11.460004882812502);
    distances.put(0.587, 11.3400048828125);
    distances.put(0.588, 11.220009765625);
    distances.put(0.589, 11.110009765625);
    distances.put(0.59, 11.00001953125);
    distances.put(0.591, 10.90001953125);
    distances.put(0.592, 10.800019531250001);
    distances.put(0.593, 10.70001953125);
    distances.put(0.594, 10.600039062499999);
    distances.put(0.595, 10.510078125);
    distances.put(0.596, 10.4300390625);
    distances.put(0.597, 10.340078125);
    distances.put(0.598, 10.260078125);
    distances.put(0.599, 10.180078125);
    distances.put(0.6, 10.100078125);
    distances.put(0.601, 10.02015625);
    distances.put(0.602, 9.95015625);
    distances.put(0.603, 9.88015625);
    distances.put(0.604, 9.81015625);
    distances.put(0.605, 9.7403125);
    distances.put(0.606, 9.6803125);
    distances.put(0.607, 9.62015625);
    distances.put(0.608, 9.5503125);
    distances.put(0.609, 9.490625);
    distances.put(0.61, 9.4403125);
    distances.put(0.611, 9.3803125);
    distances.put(0.612, 9.320625);
    distances.put(0.613, 9.2703125);
    distances.put(0.614, 9.210625);
    distances.put(0.615, 9.160625);
    distances.put(0.616, 9.110624999999999);
    distances.put(0.617, 9.060625);
    distances.put(0.618, 9.01125);
    distances.put(0.619, 8.970625);
    distances.put(0.62, 8.92125);
    distances.put(0.621, 8.880625);
    distances.put(0.622, 8.83125);
    distances.put(0.623, 8.79125);
    distances.put(0.624, 8.750625);
    distances.put(0.625, 8.70125);
    distances.put(0.626, 8.661249999999999);
    distances.put(0.627, 8.62125);
    distances.put(0.628, 8.5825);
    distances.put(0.629, 8.55125);
    distances.put(0.63, 8.51125);
    distances.put(0.631, 8.4725);
    distances.put(0.632, 8.44125);
    distances.put(0.633, 8.4025);
    distances.put(0.634, 8.37125);
    distances.put(0.635, 8.3325);
    distances.put(0.636, 8.3025);
    distances.put(0.637, 8.2725);
    distances.put(0.638, 8.24125);
    distances.put(0.639, 8.2025);
    distances.put(0.64, 8.1725);
    distances.put(0.641, 8.142500000000002);
    distances.put(0.642, 8.1125);
    distances.put(0.643, 8.085);
    distances.put(0.644, 8.0625);
    distances.put(0.645, 8.032499999999999);
    distances.put(0.646, 8.0025);
    distances.put(0.647, 7.975);
    distances.put(0.648, 7.952500000000001);
    distances.put(0.649, 7.922499999999999);
    distances.put(0.65, 7.895);
    distances.put(0.651, 7.8725000000000005);
    distances.put(0.652, 7.845);
    distances.put(0.653, 7.8225);
    distances.put(0.654, 7.795);
    distances.put(0.655, 7.775);
    distances.put(0.656, 7.7524999999999995);
    distances.put(0.657, 7.725);
    distances.put(0.658, 7.705);
    distances.put(0.659, 7.6850000000000005);
    distances.put(0.66, 7.6625);
    distances.put(0.661, 7.635);
    distances.put(0.662, 7.615);
    distances.put(0.663, 7.595);
    distances.put(0.664, 7.575);
    distances.put(0.665, 7.555);
    distances.put(0.666, 7.535);
    distances.put(0.667, 7.515);
    distances.put(0.668, 7.495);
    distances.put(0.669, 7.475);
    distances.put(0.67, 7.455);
    distances.put(0.671, 7.44);
    distances.put(0.672, 7.425);
    distances.put(0.673, 7.405);
    distances.put(0.674, 7.385);
    distances.put(0.675, 7.37);
    distances.put(0.676, 7.355);
    distances.put(0.677, 7.335);
    distances.put(0.678, 7.3149999999999995);
    distances.put(0.679, 7.3);
    distances.put(0.68, 7.285);
    distances.put(0.681, 7.27);
    distances.put(0.682, 7.255);
    distances.put(0.683, 7.235);
    distances.put(0.684, 7.22);
    distances.put(0.685, 7.205);
    distances.put(0.686, 7.19);
    distances.put(0.687, 7.175);
    distances.put(0.688, 7.16);
    distances.put(0.689, 7.15);
    distances.put(0.69, 7.135);
    distances.put(0.691, 7.12);
    distances.put(0.692, 7.105);
    distances.put(0.693, 7.09);
    distances.put(0.694, 7.08);
    distances.put(0.695, 7.0649999999999995);
    distances.put(0.696, 7.05);
    distances.put(0.697, 7.04);
    distances.put(0.698, 7.025);
    distances.put(0.699, 7.01);
    distances.put(0.7, 7.0);
    distances.put(0.701, 6.99);
    distances.put(0.702, 6.975);
    distances.put(0.703, 6.96);
    distances.put(0.704, 6.95);
    distances.put(0.705, 6.94);
    distances.put(0.706, 6.93);
    distances.put(0.707, 6.92);
    distances.put(0.708, 6.905);
    distances.put(0.709, 6.89);
    distances.put(0.71, 6.88);
    distances.put(0.711, 6.87);
    distances.put(0.712, 6.86);
    distances.put(0.713, 6.85);
    distances.put(0.714, 6.84);
    distances.put(0.715, 6.83);
    distances.put(0.716, 6.82);
    distances.put(0.717, 6.81);
    distances.put(0.718, 6.8);
    distances.put(0.719, 6.79);
    distances.put(0.72, 6.78);
    distances.put(0.721, 6.77);
    distances.put(0.722, 6.76);
    distances.put(0.723, 6.75);
    distances.put(0.724, 6.74);
    distances.put(0.725, 6.73);
    distances.put(0.726, 6.72);
    distances.put(0.727, 6.71);
    distances.put(0.728, 6.7);
    distances.put(0.729, 6.69);
    distances.put(0.73, 6.68);
    distances.put(0.732, 6.67);
    distances.put(0.733, 6.66);
    distances.put(0.734, 6.65);
    distances.put(0.735, 6.64);
    distances.put(0.736, 6.63);
    distances.put(0.737, 6.62);
    distances.put(0.739, 6.61);
    distances.put(0.74, 6.6);
    distances.put(0.741, 6.59);
    distances.put(0.742, 6.58);
    distances.put(0.744, 6.57);
    distances.put(0.745, 6.56);
    distances.put(0.746, 6.55);
    distances.put(0.748, 6.54);
    distances.put(0.749, 6.53);
    distances.put(0.751, 6.52);
    distances.put(0.752, 6.51);
    distances.put(0.753, 6.5);
    distances.put(0.755, 6.49);
    distances.put(0.756, 6.48);
    distances.put(0.758, 6.47);
    distances.put(0.759, 6.46);
    distances.put(0.761, 6.45);
    distances.put(0.763, 6.44);
    distances.put(0.764, 6.43);
    distances.put(0.766, 6.42);
    distances.put(0.767, 6.41);
    distances.put(0.769, 6.4);
    distances.put(0.771, 6.39);
    distances.put(0.773, 6.38);
    distances.put(0.774, 6.37);
    distances.put(0.776, 6.36);
    distances.put(0.778, 6.35);
    distances.put(0.78, 6.34);
    distances.put(0.782, 6.33);
    distances.put(0.784, 6.32);
    distances.put(0.786, 6.31);
    distances.put(0.788, 6.3);
    distances.put(0.79, 6.29);
    distances.put(0.792, 6.28);
    distances.put(0.794, 6.27);
    distances.put(0.796, 6.26);
    distances.put(0.798, 6.25);
    distances.put(0.801, 6.24);
    distances.put(0.803, 6.23);
    distances.put(0.805, 6.22);
    distances.put(0.808, 6.21);
    distances.put(0.81, 6.2);
    distances.put(0.813, 6.19);
    distances.put(0.815, 6.18);
    distances.put(0.818, 6.17);
    distances.put(0.821, 6.16);
    distances.put(0.824, 6.15);
    distances.put(0.826, 6.14);
    distances.put(0.829, 6.13);
    distances.put(0.833, 6.12);
    distances.put(0.836, 6.11);
    distances.put(0.839, 6.1);
    distances.put(0.842, 6.09);
    distances.put(0.846, 6.08);
    distances.put(0.849, 6.07);
    distances.put(0.853, 6.06);
    distances.put(0.857, 6.05);
    distances.put(0.861, 6.04);
    distances.put(0.865, 6.03);
    distances.put(0.869, 6.02);
    distances.put(0.873, 6.01);
    distances.put(0.878, 6.0);
    distances.put(0.883, 5.99);
    distances.put(0.888, 5.98);
    distances.put(0.893, 5.97);
    distances.put(0.899, 5.96);
    distances.put(0.905, 5.95);
    distances.put(0.911, 5.94);
    distances.put(0.918, 5.93);
    distances.put(0.925, 5.92);
    distances.put(0.933, 5.91);
    distances.put(0.941, 5.9);
    distances.put(0.951, 5.89);
    distances.put(0.961, 5.88);
    distances.put(0.973, 5.87);
    distances.put(0.988, 5.86);
    distances.put(1.006, 5.85);
    distances.put(1.032, 5.84);
    distances.put(1.075, 5.82);
    distances.put(1.079, 5.83);
    distances.put(1.134, 5.84);
    distances.put(1.167, 5.85);
    distances.put(1.192, 5.86);
    distances.put(1.214, 5.87);
    distances.put(1.233, 5.88);
    distances.put(1.251, 5.89);
    distances.put(1.268, 5.9);
    distances.put(1.283, 5.91);
    distances.put(1.299, 5.92);
    distances.put(1.313, 5.93);
    distances.put(1.327, 5.94);
    distances.put(1.341, 5.95);
    distances.put(1.354, 5.96);
    distances.put(1.367, 5.97);
    distances.put(1.38, 5.98);
    distances.put(1.392, 5.99);
    distances.put(1.404, 6.0);
    distances.put(1.416, 6.01);
    distances.put(1.428, 6.02);
    distances.put(1.44, 6.03);
    distances.put(1.452, 6.04);
    distances.put(1.463, 6.05);
    distances.put(1.474, 6.06);
    distances.put(1.485, 6.07);
    distances.put(1.496, 6.08);
    distances.put(1.507, 6.09);
    distances.put(1.518, 6.1);
    distances.put(1.529, 6.11);
    distances.put(1.54, 6.12);
    distances.put(1.55, 6.13);
    distances.put(1.561, 6.14);
    distances.put(1.571, 6.15);
    distances.put(1.582, 6.16);
    distances.put(1.592, 6.17);
    distances.put(1.602, 6.18);
    distances.put(1.613, 6.19);
    distances.put(1.623, 6.2);
    distances.put(1.633, 6.21);
    distances.put(1.643, 6.22);
    distances.put(1.653, 6.23);
    distances.put(1.663, 6.24);
    distances.put(1.673, 6.25);
    distances.put(1.683, 6.26);
    distances.put(1.693, 6.27);
    distances.put(1.703, 6.28);
    distances.put(1.713, 6.29);
    distances.put(1.722, 6.3);
    distances.put(1.732, 6.31);
    distances.put(1.742, 6.32);
    distances.put(1.752, 6.33);
    distances.put(1.761, 6.34);
    distances.put(1.771, 6.35);
    distances.put(1.781, 6.36);
    distances.put(1.79, 6.37);
    distances.put(1.8, 6.38);
    distances.put(1.81, 6.39);
    distances.put(1.819, 6.4);
    distances.put(1.829, 6.41);
    distances.put(1.838, 6.42);
    distances.put(1.848, 6.43);
    distances.put(1.857, 6.44);
    distances.put(1.867, 6.45);
    distances.put(1.876, 6.46);
    distances.put(1.886, 6.47);
    distances.put(1.895, 6.48);
    distances.put(1.905, 6.49);
    distances.put(1.914, 6.5);
    distances.put(1.923, 6.51);
    distances.put(1.933, 6.52);
    distances.put(1.942, 6.53);
    distances.put(1.952, 6.54);
    distances.put(1.961, 6.55);
    distances.put(1.97, 6.56);
    distances.put(1.98, 6.57);
    distances.put(1.989, 6.58);
    distances.put(1.999, 6.59);
    distances.put(2.008, 6.6);
    distances.put(2.017, 6.61);
    distances.put(2.027, 6.62);
    distances.put(2.036, 6.63);
    distances.put(2.045, 6.64);
    distances.put(2.055, 6.65);
    distances.put(2.064, 6.66);
    distances.put(2.073, 6.67);
    distances.put(2.083, 6.68);
    distances.put(2.092, 6.69);
    distances.put(2.101, 6.7);
    distances.put(2.111, 6.71);
    distances.put(2.12, 6.72);
    distances.put(2.129, 6.73);
    distances.put(2.139, 6.74);
    distances.put(2.148, 6.75);
    distances.put(2.157, 6.76);
    distances.put(2.167, 6.77);
    distances.put(2.176, 6.78);
    distances.put(2.185, 6.79);
    distances.put(2.195, 6.8);
    distances.put(2.204, 6.81);
    distances.put(2.213, 6.82);
    distances.put(2.222, 6.83);
    distances.put(2.232, 6.84);
    distances.put(2.241, 6.85);
    distances.put(2.25, 6.86);
    distances.put(2.26, 6.87);
    distances.put(2.269, 6.88);
    distances.put(2.278, 6.89);
    distances.put(2.288, 6.9);
    distances.put(2.297, 6.91);
    distances.put(2.306, 6.92);
    distances.put(2.316, 6.93);
    distances.put(2.325, 6.94);
    distances.put(2.334, 6.95);
    distances.put(2.344, 6.96);
    distances.put(2.353, 6.97);
    distances.put(2.362, 6.98);
    distances.put(2.372, 6.99);
    distances.put(2.381, 7.0);
    distances.put(2.39, 7.01);
    distances.put(2.4, 7.02);
    distances.put(2.409, 7.03);
    distances.put(2.419, 7.04);
    distances.put(2.428, 7.05);
    distances.put(2.437, 7.06);
    distances.put(2.447, 7.07);
    distances.put(2.456, 7.08);
    distances.put(2.465, 7.09);
    distances.put(2.475, 7.1);
    distances.put(2.484, 7.11);
    distances.put(2.494, 7.12);
    distances.put(2.503, 7.13);
    distances.put(2.512, 7.14);
    distances.put(2.522, 7.15);
    distances.put(2.531, 7.16);
    distances.put(2.541, 7.17);
    distances.put(2.55, 7.18);
    distances.put(2.56, 7.19);
    distances.put(2.569, 7.2);
    distances.put(2.578, 7.21);
    distances.put(2.588, 7.22);
    distances.put(2.597, 7.23);
    distances.put(2.607, 7.24);
    distances.put(2.616, 7.25);
    distances.put(2.626, 7.26);
    distances.put(2.635, 7.27);
    distances.put(2.645, 7.28);
    distances.put(2.654, 7.29);
    distances.put(2.664, 7.3);
    distances.put(2.673, 7.31);
    distances.put(2.683, 7.32);
    distances.put(2.692, 7.33);
    distances.put(2.702, 7.34);
    distances.put(2.711, 7.35);
    distances.put(2.721, 7.36);
    distances.put(2.73, 7.37);
    distances.put(2.74, 7.38);
    distances.put(2.749, 7.39);
    distances.put(2.759, 7.4);
    distances.put(2.769, 7.41);
    distances.put(2.778, 7.42);
    distances.put(2.788, 7.43);
    distances.put(2.797, 7.44);
    distances.put(2.807, 7.45);
    distances.put(2.817, 7.46);
    distances.put(2.826, 7.47);
    distances.put(2.836, 7.48);
    distances.put(2.845, 7.49);
    distances.put(2.855, 7.5);
    distances.put(2.865, 7.51);
    distances.put(2.874, 7.52);
    distances.put(2.884, 7.53);
    distances.put(2.894, 7.54);
    distances.put(2.903, 7.55);
    distances.put(2.913, 7.56);
    distances.put(2.923, 7.57);
    distances.put(2.932, 7.58);
    distances.put(2.942, 7.59);
    distances.put(2.952, 7.6);
    distances.put(2.961, 7.61);
    distances.put(2.971, 7.62);
    distances.put(2.981, 7.63);
    distances.put(2.991, 7.64);
    distances.put(3.0, 7.65);
    distances.put(3.01, 7.66);
    distances.put(3.02, 7.67);
    distances.put(3.03, 7.68);
    distances.put(3.039, 7.69);
    distances.put(3.049, 7.7);
    distances.put(3.059, 7.71);
    distances.put(3.069, 7.72);
    distances.put(3.079, 7.73);
    distances.put(3.088, 7.74);
    distances.put(3.098, 7.75);
    distances.put(3.108, 7.76);
    distances.put(3.118, 7.77);
    distances.put(3.128, 7.78);
    distances.put(3.138, 7.79);
    distances.put(3.147, 7.8);
    distances.put(3.157, 7.81);
    distances.put(3.167, 7.82);
    distances.put(3.177, 7.83);
    distances.put(3.187, 7.84);
    distances.put(3.197, 7.85);
    distances.put(3.207, 7.86);
    distances.put(3.217, 7.87);
    distances.put(3.227, 7.88);
    distances.put(3.237, 7.89);
    distances.put(3.247, 7.9);
    distances.put(3.257, 7.91);
    distances.put(3.266, 7.92);
    distances.put(3.276, 7.93);
    distances.put(3.286, 7.94);
    distances.put(3.296, 7.95);
    distances.put(3.306, 7.96);
    distances.put(3.316, 7.97);
    distances.put(3.326, 7.98);
    distances.put(3.336, 7.99);
    distances.put(3.346, 8.0);
    distances.put(3.357, 8.01);
    distances.put(3.367, 8.02);
    distances.put(3.377, 8.03);
    distances.put(3.387, 8.04);
    distances.put(3.397, 8.05);
    distances.put(3.407, 8.06);
    distances.put(3.417, 8.07);
    distances.put(3.427, 8.08);
    distances.put(3.437, 8.09);
    distances.put(3.447, 8.1);
    distances.put(3.457, 8.11);
    distances.put(3.468, 8.12);
    distances.put(3.478, 8.13);
    distances.put(3.488, 8.14);
    distances.put(3.498, 8.15);
    distances.put(3.508, 8.16);
    distances.put(3.518, 8.17);
    distances.put(3.529, 8.18);
    distances.put(3.539, 8.19);
    distances.put(3.549, 8.2);
    distances.put(3.559, 8.21);
    distances.put(3.569, 8.22);
    distances.put(3.58, 8.23);
    distances.put(3.59, 8.24);
    distances.put(3.6, 8.25);
    distances.put(3.61, 8.26);
    distances.put(3.621, 8.27);
    distances.put(3.631, 8.28);
    distances.put(3.641, 8.29);
    distances.put(3.652, 8.3);
    distances.put(3.662, 8.31);
    distances.put(3.672, 8.32);
    distances.put(3.683, 8.33);
    distances.put(3.693, 8.34);
    distances.put(3.703, 8.35);
    distances.put(3.714, 8.36);
    distances.put(3.724, 8.37);
    distances.put(3.734, 8.38);
    distances.put(3.745, 8.39);
    distances.put(3.755, 8.4);
    distances.put(3.766, 8.41);
    distances.put(3.776, 8.42);
    distances.put(3.786, 8.43);
    distances.put(3.797, 8.44);
    distances.put(3.807, 8.45);
    distances.put(3.818, 8.46);
    distances.put(3.828, 8.47);
    distances.put(3.839, 8.48);
    distances.put(3.849, 8.49);
    distances.put(3.86, 8.5);
    distances.put(3.87, 8.51);
    distances.put(3.881, 8.52);
    distances.put(3.891, 8.53);
    distances.put(3.902, 8.54);
    distances.put(3.912, 8.55);
    distances.put(3.923, 8.56);
    distances.put(3.933, 8.57);
    distances.put(3.944, 8.58);
    distances.put(3.954, 8.59);
    distances.put(3.965, 8.6);
    distances.put(3.976, 8.61);
    distances.put(3.986, 8.62);
    distances.put(3.997, 8.63);
    distances.put(4.007, 8.64);
    distances.put(4.018, 8.65);
    distances.put(4.029, 8.66);
    distances.put(4.039, 8.67);
    distances.put(4.05, 8.68);
    distances.put(4.061, 8.69);
    distances.put(4.071, 8.7);
    distances.put(4.082, 8.71);
    distances.put(4.093, 8.72);
    distances.put(4.104, 8.73);
    distances.put(4.114, 8.74);
    distances.put(4.125, 8.75);
    distances.put(4.136, 8.76);
    distances.put(4.146, 8.77);
    distances.put(4.157, 8.78);
    distances.put(4.168, 8.79);
    distances.put(4.179, 8.8);
    distances.put(4.19, 8.81);
    distances.put(4.2, 8.82);
    distances.put(4.211, 8.83);
    distances.put(4.222, 8.84);
    distances.put(4.233, 8.85);
    distances.put(4.244, 8.86);
    distances.put(4.254, 8.87);
    distances.put(4.265, 8.88);
    distances.put(4.276, 8.89);
    distances.put(4.287, 8.9);
    distances.put(4.298, 8.91);
    distances.put(4.309, 8.92);
    distances.put(4.32, 8.93);
    distances.put(4.331, 8.94);
    distances.put(4.342, 8.95);
    distances.put(4.352, 8.96);
    distances.put(4.363, 8.97);
    distances.put(4.374, 8.98);
    distances.put(4.385, 8.99);
    distances.put(4.396, 9.0);
    distances.put(4.407, 9.01);
    distances.put(4.418, 9.02);
    distances.put(4.429, 9.03);
    distances.put(4.44, 9.04);
    distances.put(4.451, 9.05);
    distances.put(4.462, 9.06);
    distances.put(4.473, 9.07);
    distances.put(4.484, 9.08);
    distances.put(4.495, 9.09);
    distances.put(4.507, 9.1);
    distances.put(4.518, 9.11);
    distances.put(4.529, 9.12);
    distances.put(4.54, 9.13);
    distances.put(4.551, 9.14);
    distances.put(4.562, 9.15);
    distances.put(4.573, 9.16);
    distances.put(4.584, 9.17);
    distances.put(4.595, 9.18);
    distances.put(4.607, 9.19);
    distances.put(4.618, 9.2);
    distances.put(4.629, 9.21);
    distances.put(4.64, 9.22);
    distances.put(4.651, 9.23);
    distances.put(4.663, 9.24);
    distances.put(4.674, 9.25);
    distances.put(4.685, 9.26);
    distances.put(4.696, 9.27);
    distances.put(4.707, 9.28);
    distances.put(4.719, 9.29);
    distances.put(4.73, 9.3);
    distances.put(4.741, 9.31);
    distances.put(4.753, 9.32);
    distances.put(4.764, 9.33);
    distances.put(4.775, 9.34);
    distances.put(4.787, 9.35);
    distances.put(4.798, 9.36);
    distances.put(4.809, 9.37);
    distances.put(4.821, 9.38);
    distances.put(4.832, 9.39);
    distances.put(4.843, 9.4);
    distances.put(4.855, 9.41);
    distances.put(4.866, 9.42);
    distances.put(4.877, 9.43);
    distances.put(4.889, 9.44);
    distances.put(4.9, 9.45);
    distances.put(4.912, 9.46);
    distances.put(4.923, 9.47);
    distances.put(4.935, 9.48);
    distances.put(4.946, 9.49);
    distances.put(4.957, 9.5);
    distances.put(4.969, 9.51);
    distances.put(4.98, 9.52);
    distances.put(4.992, 9.53);
    distances.put(5.003, 9.54);
    distances.put(5.015, 9.55);
    distances.put(5.026, 9.56);
    distances.put(5.038, 9.57);
    distances.put(5.05, 9.58);
    distances.put(5.061, 9.59);
    distances.put(5.073, 9.6);
    distances.put(5.084, 9.61);
    distances.put(5.096, 9.62);
    distances.put(5.107, 9.63);
    distances.put(5.119, 9.64);
    distances.put(5.131, 9.65);
    distances.put(5.142, 9.66);
    distances.put(5.154, 9.67);
    distances.put(5.166, 9.68);
    distances.put(5.177, 9.69);
    distances.put(5.189, 9.7);
    distances.put(5.201, 9.71);
    distances.put(5.212, 9.72);
    distances.put(5.224, 9.73);
    distances.put(5.236, 9.74);
    distances.put(5.247, 9.75);
    distances.put(5.259, 9.76);
    distances.put(5.271, 9.77);
    distances.put(5.283, 9.78);
    distances.put(5.294, 9.79);
    distances.put(5.306, 9.8);
    distances.put(5.318, 9.81);
    distances.put(5.33, 9.82);
    distances.put(5.341, 9.83);
    distances.put(5.353, 9.84);
    distances.put(5.365, 9.85);
    distances.put(5.377, 9.86);
    distances.put(5.389, 9.87);
    distances.put(5.401, 9.88);
    distances.put(5.412, 9.89);
    distances.put(5.424, 9.9);
    distances.put(5.436, 9.91);
    distances.put(5.448, 9.92);
    distances.put(5.46, 9.93);
    distances.put(5.472, 9.94);
    distances.put(5.484, 9.95);
    distances.put(5.496, 9.96);
    distances.put(5.508, 9.97);
    distances.put(5.52, 9.98);
    distances.put(5.531, 9.99);
    distances.put(5.543, 10.0);
    distances.put(5.555, 10.01);
    distances.put(5.567, 10.02);
    distances.put(5.579, 10.03);
    distances.put(5.591, 10.04);
    distances.put(5.603, 10.05);
    distances.put(5.615, 10.06);
    distances.put(5.627, 10.07);
    distances.put(5.64, 10.08);
    distances.put(5.652, 10.09);
    distances.put(5.664, 10.1);
    distances.put(5.676, 10.11);
    distances.put(5.688, 10.12);
    distances.put(5.7, 10.13);
    distances.put(5.712, 10.14);
    distances.put(5.724, 10.15);
    distances.put(5.736, 10.16);
    distances.put(5.748, 10.17);
    distances.put(5.761, 10.18);
    distances.put(5.773, 10.19);
    distances.put(5.785, 10.2);
    distances.put(5.797, 10.21);
    distances.put(5.809, 10.22);
    distances.put(5.821, 10.23);
    distances.put(5.834, 10.24);
    distances.put(5.846, 10.25);
    distances.put(5.858, 10.26);
    distances.put(5.87, 10.27);
    distances.put(5.883, 10.28);
    distances.put(5.895, 10.29);
    distances.put(5.907, 10.3);
    distances.put(5.919, 10.31);
    distances.put(5.932, 10.32);
    distances.put(5.944, 10.33);
    distances.put(5.956, 10.34);
    distances.put(5.969, 10.35);
    distances.put(5.981, 10.36);
    distances.put(5.993, 10.37);
    distances.put(6.006, 10.38);
    distances.put(6.018, 10.39);
    distances.put(6.03, 10.4);
    distances.put(6.043, 10.41);
    distances.put(6.055, 10.42);
    distances.put(6.067, 10.43);
    distances.put(6.08, 10.44);
    distances.put(6.092, 10.45);
    distances.put(6.105, 10.46);
    distances.put(6.117, 10.47);
    distances.put(6.13, 10.48);
    distances.put(6.142, 10.49);
    distances.put(6.155, 10.5);
    distances.put(6.167, 10.51);
    distances.put(6.18, 10.52);
    distances.put(6.192, 10.53);
    distances.put(6.205, 10.54);
    distances.put(6.217, 10.55);
    distances.put(6.23, 10.56);
    distances.put(6.242, 10.57);
    distances.put(6.255, 10.58);
    distances.put(6.267, 10.59);
    distances.put(6.28, 10.6);
    distances.put(6.292, 10.61);
    distances.put(6.305, 10.62);
    distances.put(6.318, 10.63);
    distances.put(6.33, 10.64);
    distances.put(6.343, 10.65);
    distances.put(6.355, 10.66);
    distances.put(6.368, 10.67);
    distances.put(6.381, 10.68);
    distances.put(6.393, 10.69);
    distances.put(6.406, 10.7);
    distances.put(6.419, 10.71);
    distances.put(6.431, 10.72);
    distances.put(6.444, 10.73);
    distances.put(6.457, 10.74);
    distances.put(6.47, 10.75);
    distances.put(6.482, 10.76);
    distances.put(6.495, 10.77);
    distances.put(6.508, 10.78);
    distances.put(6.521, 10.79);
    distances.put(6.533, 10.8);
    distances.put(6.546, 10.81);
    distances.put(6.559, 10.82);
    distances.put(6.572, 10.83);
    distances.put(6.585, 10.84);
    distances.put(6.597, 10.85);
    distances.put(6.61, 10.86);
    distances.put(6.623, 10.87);
    distances.put(6.636, 10.88);
    distances.put(6.649, 10.89);
    distances.put(6.662, 10.9);
    distances.put(6.674, 10.91);
    distances.put(6.687, 10.92);
    distances.put(6.7, 10.93);
    distances.put(6.713, 10.94);
    distances.put(6.726, 10.95);
    distances.put(6.739, 10.96);
    distances.put(6.752, 10.97);
    distances.put(6.765, 10.98);
    distances.put(6.778, 10.99);
    distances.put(6.791, 11.0);
    distances.put(6.804, 11.01);
    distances.put(6.817, 11.02);
    distances.put(6.83, 11.03);
    distances.put(6.843, 11.04);
    distances.put(6.856, 11.05);
    distances.put(6.869, 11.06);
    distances.put(6.882, 11.07);
    distances.put(6.895, 11.08);
    distances.put(6.908, 11.09);
    distances.put(6.921, 11.1);
    distances.put(6.934, 11.11);
    distances.put(6.947, 11.12);
    distances.put(6.96, 11.13);
    distances.put(6.974, 11.14);
    distances.put(6.987, 11.15);
    distances.put(7.0, 11.16);
    distances.put(7.013, 11.17);
    distances.put(7.026, 11.18);
    distances.put(7.039, 11.19);
    distances.put(7.052, 11.2);
    distances.put(7.066, 11.21);
    distances.put(7.079, 11.22);
    distances.put(7.092, 11.23);
    distances.put(7.105, 11.24);
    distances.put(7.118, 11.25);
    distances.put(7.132, 11.26);
    distances.put(7.145, 11.27);
    distances.put(7.158, 11.28);
    distances.put(7.171, 11.29);
    distances.put(7.185, 11.3);
    distances.put(7.198, 11.31);
    distances.put(7.211, 11.32);
    distances.put(7.225, 11.33);
    distances.put(7.238, 11.34);
    distances.put(7.251, 11.35);
    distances.put(7.265, 11.36);
    distances.put(7.278, 11.37);
    distances.put(7.291, 11.38);
    distances.put(7.305, 11.39);
    distances.put(7.318, 11.4);
    distances.put(7.331, 11.41);
    distances.put(7.345, 11.42);
    distances.put(7.358, 11.43);
    distances.put(7.372, 11.44);
    distances.put(7.385, 11.45);
    distances.put(7.398, 11.46);
    distances.put(7.412, 11.47);
    distances.put(7.425, 11.48);
    distances.put(7.439, 11.49);
    distances.put(7.452, 11.5);
    distances.put(7.466, 11.51);
    distances.put(7.479, 11.52);
    distances.put(7.493, 11.53);
    distances.put(7.506, 11.54);
    distances.put(7.52, 11.55);
    distances.put(7.533, 11.56);
    distances.put(7.547, 11.57);
    distances.put(7.56, 11.58);
    distances.put(7.574, 11.59);
    distances.put(7.588, 11.6);
    distances.put(7.601, 11.61);
    distances.put(7.615, 11.62);
    distances.put(7.628, 11.63);
    distances.put(7.642, 11.64);
    distances.put(7.656, 11.65);
    distances.put(7.669, 11.66);
    distances.put(7.683, 11.67);
    distances.put(7.697, 11.68);
    distances.put(7.71, 11.69);
    distances.put(7.724, 11.7);
    distances.put(7.738, 11.71);
    distances.put(7.751, 11.72);
    distances.put(7.765, 11.73);
    distances.put(7.779, 11.74);
    distances.put(7.792, 11.75);
    distances.put(7.806, 11.76);
    distances.put(7.82, 11.77);
    distances.put(7.834, 11.78);
    distances.put(7.847, 11.79);
    distances.put(7.861, 11.8);
    distances.put(7.875, 11.81);
    distances.put(7.889, 11.82);
    distances.put(7.903, 11.83);
    distances.put(7.916, 11.84);
    distances.put(7.93, 11.85);
    distances.put(7.944, 11.86);
    distances.put(7.958, 11.87);
    distances.put(7.972, 11.88);
    distances.put(7.986, 11.89);
    distances.put(7.999, 11.9);
    distances.put(8.013, 11.91);
    distances.put(8.027, 11.92);
    distances.put(8.041, 11.93);
    distances.put(8.055, 11.94);
    distances.put(8.069, 11.95);
    distances.put(8.083, 11.96);
    distances.put(8.097, 11.97);
    distances.put(8.111, 11.98);
    distances.put(8.125, 11.99);
    distances.put(8.139, 12.0);
    distances.put(8.153, 12.01);
    distances.put(8.167, 12.02);
    distances.put(8.181, 12.03);
    distances.put(8.195, 12.04);
    distances.put(8.209, 12.05);
    distances.put(8.223, 12.06);
    distances.put(8.237, 12.07);
    distances.put(8.251, 12.08);
    distances.put(8.265, 12.09);
    distances.put(8.279, 12.1);
    distances.put(8.293, 12.11);
    distances.put(8.307, 12.12);
    distances.put(8.321, 12.13);
    distances.put(8.335, 12.14);
    distances.put(8.349, 12.15);
    distances.put(8.364, 12.16);
    distances.put(8.378, 12.17);
    distances.put(8.392, 12.18);
    distances.put(8.406, 12.19);
    distances.put(8.42, 12.2);
    distances.put(8.434, 12.21);
    distances.put(8.449, 12.22);
    distances.put(8.463, 12.23);
    distances.put(8.477, 12.24);
    distances.put(8.491, 12.25);
    distances.put(8.505, 12.26);
    distances.put(8.52, 12.27);
    distances.put(8.534, 12.28);
    distances.put(8.548, 12.29);
    distances.put(8.563, 12.3);
    distances.put(8.577, 12.31);
    distances.put(8.591, 12.32);
    distances.put(8.605, 12.33);
    distances.put(8.62, 12.34);
    distances.put(8.634, 12.35);
    distances.put(8.648, 12.36);
    distances.put(8.663, 12.37);
    distances.put(8.677, 12.38);
    distances.put(8.691, 12.39);
    distances.put(8.706, 12.4);
    distances.put(8.72, 12.41);
    distances.put(8.735, 12.42);
    distances.put(8.749, 12.43);
    distances.put(8.763, 12.44);
    distances.put(8.778, 12.45);
    distances.put(8.792, 12.46);
    distances.put(8.807, 12.47);
    distances.put(8.821, 12.48);
    distances.put(8.836, 12.49);
    distances.put(8.85, 12.5);
    distances.put(8.864, 12.51);
    distances.put(8.879, 12.52);
    distances.put(8.893, 12.53);
    distances.put(8.908, 12.54);
    distances.put(8.922, 12.55);
    distances.put(8.937, 12.56);
    distances.put(8.952, 12.57);
    distances.put(8.966, 12.58);
    distances.put(8.981, 12.59);
    distances.put(8.995, 12.6);
    distances.put(9.01, 12.61);
    distances.put(9.024, 12.62);
    distances.put(9.039, 12.63);
    distances.put(9.054, 12.64);
    distances.put(9.068, 12.65);
    distances.put(9.083, 12.66);
    distances.put(9.097, 12.67);
    distances.put(9.112, 12.68);
    distances.put(9.127, 12.69);
    distances.put(9.141, 12.7);
    distances.put(9.156, 12.71);
    distances.put(9.171, 12.72);
    distances.put(9.186, 12.73);
    distances.put(9.2, 12.74);
    distances.put(9.215, 12.75);
    distances.put(9.23, 12.76);
    distances.put(9.244, 12.77);
    distances.put(9.259, 12.78);
    distances.put(9.274, 12.79);
  }

  public double getShooterSpeed(double distance)
  {
    distance = (int)(distance*1000)/(1000.0);
    if(distances.containsKey(distance))
    {
      return distances.get(distance);
    }
    else if(distance > 0.6 && distance < 9.2)
    {
      double lowKey = distances.floorKey(distance),highKey = distances.ceilingKey(distance);
      return (distances.get(lowKey) + distances.get(highKey))/2;
    }
    return -1;
  }

  public void fileTesting(MatOfPoint contour, String measuredDistance) throws IOException {
    FileWriter fw = new FileWriter("/home/lvuser/distanceData/output.txt");    
    fw.write(measuredDistance); 
    System.out.println(measuredDistance);
    SmartDashboard.putString("measured distance", measuredDistance);
    fw.close();    
  }
  @Override
  public void robotInit() {
    encoder = new Encoder(0, 1);

    //leftSide.set(DoubleSolenoid.Value.kForward);
    //rightSide.set(DoubleSolenoid.Value.kForward);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    AnalogInput.setGlobalSampleRate(100.0);
    //SmartDashboard.putNumber("Ultrasonic Sensor 0", 5.0 * m_ultrasonic0.getVoltage() / mvPer5mm);
    pipeline = new GripPipeline();
    initializeDistanceTable();

    m_oi = new OI();

    m_visionThread = new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
      img = new Mat();
      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(img) == 0) {
          outputStream.notifyError(cvSink.getError());
          continue;
        }
        // m_oi.autoAlignButton.whenPressed(new AlignShooter(img));
        // new AlignShooter(img);
        Robot.pipeline.process(img);
        ArrayList<MatOfPoint> contours = Robot.pipeline.filterContoursOutput();
        if (contours.size() > 0) {
          MatOfPoint contour = getLargestContour(contours);
          //System.out.println("Vision Distances: "+ visionDistanceWidth(contour)+" "+ visionDistanceHeight(contour)+" "+averageVisionDistance(contour));      
          double a =1;
          //String measuredDistance = Double.toString(averageVisionDistance(contour));
          //double realDist = SmartDashboard.getNumber("inputted real dist", a);
          System.out.println(visionDistanceHeight(contour)+" "+visionDistanceWidth(contour)+" "+averageVisionDistance(contour));
          /*if (realDist != 0) {
            try {
              fileTesting(contour, measuredDistance);
            }
            catch (IOException e) {
              System.out.println("no new file created");
            }
          }*/
          int center[] = findCenter(contour);
          Imgproc.circle(img, new Point(center[0], center[1]), 10, new Scalar(255, 255, 0), 10);
          Rect boundingRect = Imgproc.boundingRect(contour);
          ArrayList<MatOfPoint> temp = new ArrayList<MatOfPoint>();
          temp.add(contour);
          Imgproc.drawContours(img, temp, 0, new Scalar(255, 0, 0), 5);
        }
        Imgproc.circle(img, new Point(imgWidth / 2, imgHeight / 2), 10, new Scalar(255, 255, 0), 10);
        outputStream.putFrame(img);

      }

      
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    climb1 = new WPI_VictorSPX(9); // done
    climb2 = new WPI_VictorSPX(10); // done

    storage = new WPI_VictorSPX(12); // done
    intake = new WPI_VictorSPX(8); // done

    arm1 = new WPI_TalonSRX(5); // done
    arm1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    arm1.setSelectedSensorPosition(0);
    arm2 = new WPI_VictorSPX(11); // done
    arm1.setSelectedSensorPosition(0);

    shoot1 = new WPI_TalonSRX(6); //done
    shoot2 = new WPI_VictorSPX(7); // done

    shoot1.configPeakCurrentLimit(60);
    shoot1.configContinuousCurrentLimit(28);

    drive1 = new WPI_TalonFX(1); //done
    slave1 = new WPI_TalonFX(2); //done
    drive2 = new WPI_TalonFX(3); // done
    slave2 = new WPI_TalonFX(4); //done

    turret = new WPI_VictorSPX(13); // not done yet

    m_turret = new Turret(turret);

    m_drive = new DifferentialDrive(drive1, drive2);
    SmartDashboard.putData(m_drive);
    m_drivetrain = new DriveTrain();

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
    //cont++; 
    
    //Scheduler.getInstance().run();
    //System.out.println(arm1.getSelectedSensorPosition(0));
    
    /*if(Math.abs(arm1.getSelectedSensorPosition(0))<4096)
    {
      arm1.set(0.1);
    }
    else
    {
      arm1.set(0);
    }*/

    shoot1.set(-0.15);
    //shoot2.set(-0.15);
    
    //System.out.println(arm1.getSelectedSensorPosition(0));
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
