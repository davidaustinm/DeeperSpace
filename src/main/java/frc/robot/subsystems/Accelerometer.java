/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.AccelerometerCommand;

/**
 * Add your docs here.
 */
public class Accelerometer extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  BuiltInAccelerometer acc = new BuiltInAccelerometer();
  boolean jolt = false;

  public void setJolt(boolean b) {
    jolt = b;
  }

  public boolean getJolt() {
    return jolt;
  }

  public double getX() {
    return acc.getX();
  }

  public double getY() {
    return acc.getY();
  }

  public double getZ() {
    return acc.getZ();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new AccelerometerCommand());
  }
}
