/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.TwoSparkDriveCommand;

/**
 * Add your docs here.
 */
public class TwoSparkDrivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax right1 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);
  
  public TwoSparkDrivetrain() {
    right1.setInverted(true);
    right2.setInverted(true);
  }

  public void setPower(double left, double right) {
    left1.set(left);
    left2.set(left);
    right1.set(right);
    right2.set(right);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TwoSparkDriveCommand());
  }
}
