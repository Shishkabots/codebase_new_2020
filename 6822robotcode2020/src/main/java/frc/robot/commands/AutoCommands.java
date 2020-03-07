package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCommands extends CommandGroup {
    public AutoCommands() {
        addSequential(new ShooterMacro());
        addSequential(new PassInitiationLine());
    }
}