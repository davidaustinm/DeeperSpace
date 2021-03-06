/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.DriveToTarget2;

public class Sensors extends Subsystem {
  AHRS navx;
  double gyroOffset = 0;
  double cutPoint = 180;
  double[] driveEncoderOffsets = new double[] {0,0};
  double pitchOffset = 0;
  //DigitalInput vacSense = new DigitalInput(RobotMap.VAC_SENSE);
  //public final double ENCODERCOUNTSPERINCH = 0.8; // wooden robot
  public final double ENCODER_COUNTS_PER_INCH_HIGH_GEAR = 0.44444; // new Drive train no extras
  public final double ENCODER_COUNTS_PER_INCH_LOW_GEAR = .63;
  public static final double NEW_HIGH_ENCODER_COUNTS = 0.63;
  public static final double NEW_LOW_ENCODER_COUNTS = 0.63/2.0;
  public static final double ENCODER_TICKS_PER_INCH = 23.8; // powerup robot
  double positionX = 0;
  double positionY = 0;
  double[] lastDriveEncoder = new double[] {0,0};
  //DriveToTarget driveToTarget = null;
  AutoTarget driveToTarget = null;
  boolean drivingToTarget = false;

  public Sensors() {
    navx = new AHRS(I2C.Port.kMXP);
  }

  public void setDrivingToTarget(boolean b) {
    drivingToTarget = b;
  }

  public boolean isDrivingToTarget() {
    return drivingToTarget;
  }

  /*
  public boolean getVacSense() {
    return vacSense.get();
  }
  */

  public double getEncoderCountsPerInch() {
    return Robot.pneumatics.getState(Pneumatics.SHIFT) ? NEW_LOW_ENCODER_COUNTS : NEW_HIGH_ENCODER_COUNTS;
  }

  public void setDriveToTarget(AutoTarget dt) {
    driveToTarget = dt;
  }

  public void driveToTargetOff() {
    if(driveToTarget == null){
      return;
    }
    driveToTarget.cancel();
    driveToTarget = null;
  }

  public void resetPitch() {
    pitchOffset = navx.getPitch();
  }

  public double getPitch() {
    return navx.getRoll() - pitchOffset;
  }

  public int getIntakeRotatePosition() {
    return Robot.intakeRotate.getPosition();
  }

  public double readGyro() {
    return -navx.getAngle();
  }

  public void resetGyro() {
    gyroOffset = readGyro();
  }

  public void setGyro(double degrees) {
    gyroOffset = readGyro() - degrees;
  }

  public double getHeading() {
    double heading = readGyro() - gyroOffset;
    if (Robot.driveTrain.isSwitched()) heading += 180;
    while (heading > cutPoint) heading -= 360;
    while (heading < cutPoint - 360) heading += 360;
    return heading;
  }

  public void setCutpoint(double cp) {
    cutPoint = cp;
  }

  public double[] readDriveEncoders() {
    return Robot.driveTrain.getDriveEncoders();
  }

  public void resetDriveEncoders() {
    driveEncoderOffsets = readDriveEncoders();
    lastDriveEncoder = new double[] {0,0};
  }

  public double[] getDriveEncoders() {
    double[] encoderReadings = readDriveEncoders();
    return new double[] {encoderReadings[0] - driveEncoderOffsets[0],
                      encoderReadings[1]-driveEncoderOffsets[1]};
  }

  public double getAverageDriveEncoder() {
    double[] encoders = getDriveEncoders();
    return (encoders[0] + encoders[1]) / 2.0;
  }

  public double getLeftDriveEncoder() {
    double[] encoderReadings = readDriveEncoders();
    return encoderReadings[0];
  }
  public double getRightDriveEncoder() {
    double[] encoderReadings = readDriveEncoders();
    return encoderReadings[1];
  }
  public void resetPosition() {
    positionX = 0;
    positionY = 0;
  }
  public void updatePosition() {
    double[] driveEncoders = getDriveEncoders();
    double changeInLeft = driveEncoders[0] - lastDriveEncoder[0];
    double changeInRight = driveEncoders[1] - lastDriveEncoder[1];
    double distance = (changeInLeft + changeInRight) / 2;
    double heading = Math.toRadians(Robot.sensors.getHeading());
    positionX += distance * Math.cos(heading);
    positionY += distance * Math.sin(heading);
    lastDriveEncoder = driveEncoders;
  }
  public double[] getPosition() {
    return new double[] {positionX / ENCODER_COUNTS_PER_INCH_LOW_GEAR, positionY / ENCODER_COUNTS_PER_INCH_LOW_GEAR };
  }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
