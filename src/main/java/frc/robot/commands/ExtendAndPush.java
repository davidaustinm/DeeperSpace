/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Pneumatics;

public class ExtendAndPush extends Command {
  Timer timer;
  boolean finished = false;
  long stopTime;
  public ExtendAndPush() {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.pneumatics);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //timer = new Timer();
    //timer.start();
    stopTime = System.currentTimeMillis() + 1000;
    Robot.pneumatics.setState(Pneumatics.PUSHER, true);
    Robot.pneumatics.setState(Pneumatics.PANEL_PUSHER, true);
    //System.out.println("Starting pusher");
    finished = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //System.out.println(stopTime + " " + System.currentTimeMillis());
    if (System.currentTimeMillis() > stopTime) {
      Robot.pneumatics.setState(Pneumatics.PANEL_PUSHER, false);
      finished = true;
      //System.out.println("finished pusher");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
