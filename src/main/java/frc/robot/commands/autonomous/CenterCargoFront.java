/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class CenterCargoFront extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CenterCargoFront() {
    addParallel(new ReadyIntake());
    addSequential(new Wait(750));
    addSequential(new DriveForwardForDistance(40, 0.4, 0));
    addSequential(new DriveToTarget3());
    //addSequential(new DriveForwardForDistance(2.5, 0.2, 0));
    //addSequential(new ExtendAndPush());
  
  }
}
