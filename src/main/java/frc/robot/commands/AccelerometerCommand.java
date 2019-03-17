/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AccelerometerCommand extends Command {
  public AccelerometerCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.accelerometer);
  }

  int numValues = 5;
  double[][] values = new double[3][numValues];
  int count = 0;
  double[] averages = new double[3];

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    values[0][count] = Robot.accelerometer.getX();
    values[1][count] = Robot.accelerometer.getY();
    values[2][count] = Robot.accelerometer.getZ();
    count ++;
    if (count >= numValues) count = 0;
    for (int i = 0; i < 3; i++) {
      averages[i] = 0;
      for (int j = 0; j < numValues; j++) {
        averages[i] += values[i][j];
      }
      averages[i] /= numValues;
    }

    boolean jolt = false;
    if (Math.abs(averages[0]) > 0.25) jolt = true;
    if (Math.abs(averages[1]) > 0.25) jolt = true;
    Robot.accelerometer.setJolt(jolt);
    SmartDashboard.putNumber("x", averages[0]);
    SmartDashboard.putNumber("y", averages[1]);
    //SmartDashboard.putNumber("z", Robot.accelerometer.getZ());
    
  }

  public boolean jolt() {
    if (Math.abs(averages[0]) > 0.5) return true;
    if (Math.abs(averages[1]) > 0.5) return true;
    return false;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
