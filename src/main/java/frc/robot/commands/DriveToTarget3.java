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
import frc.robot.subsystems.Sensors;
import frc.robot.utilities.Utilities;

public class DriveToTarget3 extends Command {
  Timer endTimer, neverSeenTimer;
  double encoderTarget;
  double remainingDistance=9999;
  
  double maxEndTime = 1.5;
  double maxNeverSeenTime = 2.0;
  double targetOff = 45;
  double rampDown = 40;
  double kAngle = 0.005;
  double clipAnglePct = 0.2;
  double speed = 0.3;
  double stopWithJolt = 25;
  
  public DriveToTarget3() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.sensors.setDrivingToTarget(true);;
    Robot.pneumatics.setState(Pneumatics.SHIFT, false);
    endTimer = null;
    neverSeenTimer = null;
    double distance = Robot.targetInfo.getDistance();
    if (Double.isNaN(distance)) {
      encoderTarget = Double.MAX_VALUE;
      remainingDistance = Double.MAX_VALUE;
      neverSeenTimer = new Timer();
      neverSeenTimer.start();
    } else {
      double position = Robot.sensors.getAverageDriveEncoder();
      encoderTarget = position + distance * Sensors.NEW_HIGH_ENCODER_COUNTS;
      remainingDistance = distance;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // how much further do we have to go?
    double position = Robot.sensors.getAverageDriveEncoder();
    remainingDistance = (encoderTarget - position) / Sensors.NEW_HIGH_ENCODER_COUNTS;

    // we are getting close so don't read the camera anymore
    // drive straight with decreasing power, start endTimer
    if (remainingDistance < targetOff) {
      double currentSpeed = speed;
      if (remainingDistance < rampDown) {
        currentSpeed *= remainingDistance / rampDown * 0.75;
        if (endTimer == null) {
          endTimer = new Timer();
          endTimer.start();
        }
      }
      Robot.driveTrain.setPower(currentSpeed, currentSpeed);
      return;
    }

    // we're still pretty far away so read camera
    double distance = Robot.targetInfo.getDistance();

    // no sighting so drive straight
    if (Double.isNaN(distance)) {
      Robot.driveTrain.setPower(speed, speed);
      return;
    }
    if (neverSeenTimer != null) {
      neverSeenTimer.stop();
      neverSeenTimer = null;
    }
    remainingDistance = distance;
    encoderTarget = position + distance * Sensors.NEW_HIGH_ENCODER_COUNTS;
    double correction = kAngle * Robot.targetInfo.getAngle();
    correction = Utilities.clip(correction, -clipAnglePct*speed, clipAnglePct*speed);
    Robot.driveTrain.setPower(speed - correction, speed + correction);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //System.out.println(remainingDistance);
    if (remainingDistance < 18) {
      System.out.println("remaining time");
      return true;
    }
    if (neverSeenTimer != null && neverSeenTimer.get() > maxNeverSeenTime) {
      System.out.println("never seen");
      return true;
    }
    if (endTimer != null && endTimer.get() > maxEndTime) {
      System.out.println("end time");
      return true;
    }
    if (remainingDistance < stopWithJolt && Robot.accelerometer.getJolt()) {
      System.out.println("jolt " + remainingDistance);
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.sensors.setDrivingToTarget(false);
    endTimer = null;
    neverSeenTimer = null;
    Robot.driveTrain.setPower(0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
