package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class OI {
    public Joystick controllerOne = new Joystick(0);
    public Joystick controllerTwo = new Joystick(1); //also an xbox
    public Button boost = new JoystickButton(controllerOne, 6);
    public Button alignButton = new JoystickButton(controllerOne, 4); // Y 
    public Button setReverse = new JoystickButton(controllerOne, 3); // X
    public Button toggleCoast = new JoystickButton(controllerOne, 1); // A
    public Button toggleBrake = new JoystickButton(controllerOne, 2); // B
    public Button stopRobot = new JoystickButton(controllerOne, 10); // click the right joystick
    public Button stopTurning = new JoystickButton(controllerOne, 23);
    public Button turn180 = new JoystickButton(controllerOne, 9); // click the left joystick
    public Button climbButton = new JoystickButton(controllerTwo, 4); // Y
    public Button flashColor = new JoystickButton(controllerTwo, 1); // A
  
    public OI() {
      climbButton.whenPressed(new ActivateClimb());
      alignButton.whenPressed(new AlignShooter(Robot.img)); // includes all of the commands in the align shooter command group
      toggleCoast.whenPressed(new ToggleCoastMode());
      toggleBrake.whenPressed(new ToggleBrakeMode());
      stopRobot.whenPressed(new StopRobot());
      stopTurning.whenPressed(new StopTurning());
      flashColor.whenPressed(new Flash());

    }
}