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

public class TurnTurret extends Command {
  public TurnTurret() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_turret);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_turret.rotate(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double turnAxis = Robot.m_oi.controllerOne.getRawAxis(4);
    //System.out.println(turnAxis);
    double turnCoef = 0.4;
    Robot.m_turret.rotate(turnCoef * turnAxis);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_turret.rotate(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_turret.rotate(0);
  }
}
