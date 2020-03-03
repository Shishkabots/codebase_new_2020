package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Robot;

public class OI {
    public Joystick controllerOne = new Joystick(0);
    public Joystick controllerTwo = new Joystick(1);

    public Button boost = new JoystickButton(controllerOne, 6); // RB
    public Button setReverse = new JoystickButton(controllerOne, 3); // X
    public Button toggleCoast = new JoystickButton(controllerOne, 1); // A
    public Button toggleBrake = new JoystickButton(controllerOne, 2); // B
    public Button stopRobot = new JoystickButton(controllerOne, 10); // click the right joystick
    public Button stopTurning = new JoystickButton(controllerOne, 23);
    public Button turn180 = new JoystickButton(controllerOne, 9); // click the left joystick

    public Button climbButton = new JoystickButton(controllerTwo, 4); // Y
    public Button flashColor = new JoystickButton(controllerTwo, 1); // A
    public Button turretAlignButton = new JoystickButton(controllerTwo, 3); // X

    // controller axes:
    // controllerOne: 
      // DriveTrainControl.java:
        // double lTrigger = Robot.m_oi.controllerOne.getRawAxis(2);
        // double rTrigger = Robot.m_oi.controllerOne.getRawAxis(3);
        // double turnAxis = Robot.m_oi.controllerOne.getRawAxis(4);
    // controllerTwo:
      // TurnIntake.java:
        // double lTrigger = Robot.m_oi.controllerTwo.getRawAxis(2);
        // double rTrigger = Robot.m_oi.controllerTwo.getRawAxis(3);
      // TurnTurret.java:
        // double turnAxis = Robot.m_oi.controllerTwo.getRawAxis(4);
      // ManualShooter.java:
        // double voltageAxis = Robot.m_oi.controllerTwo.getRawAxis(1);

  
    public OI() {
      climbButton.whenPressed(new ActivateClimb());
      toggleCoast.whenPressed(new ToggleCoastMode());
      toggleBrake.whenPressed(new ToggleBrakeMode());
      stopRobot.whenPressed(new StopRobot());
      stopTurning.whenPressed(new StopTurning());
      flashColor.whenPressed(new Flash());
      turretAlignButton.whenPressed(new ShooterMacro()); // includes all of the commands in the align shooter command group
    }
}