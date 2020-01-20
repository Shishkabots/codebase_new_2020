package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class OI {
    public Joystick controllerOne = new Joystick(0);
    public Button boost = new JoystickButton(controllerOne, 6); 
    public Button autoAlignButton = new JoystickButton(controllerOne, 4); // Y added this for 2020 season
    public Button setReverse = new JoystickButton(controllerOne, 3); // X
    public Button toggleCoast = new JoystickButton(controllerOne, 1); // A
    public Button toggleBrake = new JoystickButton(controllerOne, 2); // B
    public Button cancel = new JoystickButton(controllerOne, 10); // click the right joystick
    public Button turn180 = new JoystickButton(controllerOne, 9); // click the left joystick
    public Joystick controllerTwo = new Joystick(1); //also an xbox
    public Button flashcolor = new JoystickButton(controllerTwo, 1); // A
    // cargo turning is axis/trigger control in the TurnCargo command
  
    public OI() {
    }
}