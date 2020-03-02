/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualShooter extends Command {
  public ManualShooter() {
    requires(Robot.m_shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_shooter.shoot(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double leadAxis = Robot.m_oi.controllerTwo.getRawAxis(1);
    double slaveAxis = Robot.m_oi.controllerTwo.getRawAxis(0);

    // don't want the two motors to stall against each other; only let the higher voltage one spin
    // tried disabling one if they are in opposite direction, but this doesn't seem to work well
    if (Math.abs(leadAxis) > Math.abs(slaveAxis)) {
      Robot.m_shooter.shootLead(leadAxis);
    }
    else {
      Robot.m_shooter.shootSlave(slaveAxis);
    }
    

    // double voltageAxis = Robot.m_oi.controllerTwo.getRawAxis(1);
    // Robot.m_shooter.shoot(voltageAxis);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_shooter.shoot(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_shooter.shoot(0);
  }
}
