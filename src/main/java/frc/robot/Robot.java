/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveToJolt;
import frc.robot.commands.GameState;
import frc.robot.commands.TwoSparkDriveCommand;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.Accelerometer;
import frc.robot.subsystems.AutoSwitches;
import frc.robot.subsystems.DeepSpaceDriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FrontLiftMotors;
import frc.robot.subsystems.IntakeRollerMotors;
import frc.robot.subsystems.IntakeRotateMotors;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.TwoSparkDrivetrain;
import frc.robot.subsystems.VelocityRecord;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.RearLiftDriveMotors;
import frc.robot.subsystems.RearLiftMotors;
import frc.robot.utilities.ReadCameraPorts;
import frc.robot.utilities.TCPClient;
//import frc.robot.utilities.TCPClient;
import frc.robot.utilities.TargetCamera;
import frc.robot.utilities.TargetInfo;
import frc.robot.utilities.UDPServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI oi;
  public static DeepSpaceDriveTrain driveTrain = new DeepSpaceDriveTrain();
  public static Accelerometer accelerometer = new Accelerometer();
  public static TwoSparkDrivetrain twoSparkDrivetrain; // = new TwoSparkDrivetrain();
  public static IntakeRollerMotors intakeRoller = new IntakeRollerMotors();
  public static IntakeRotateMotors intakeRotate = new IntakeRotateMotors();
  public static FrontLiftMotors frontLift = new FrontLiftMotors();
  public static RearLiftMotors rearLift = new RearLiftMotors();
  public static RearLiftDriveMotors rearLiftDrive = new RearLiftDriveMotors();
  public static Sensors sensors = new Sensors();
  public static TargetCamera camera;
  // public static TCPClient client = null;
  public static UDPServer client = null;
  public static Pneumatics pneumatics = new Pneumatics();
  public static GameState gameState = new GameState();
  public static TargetInfo targetInfo;
  //public static VelocityRecord velocityRecord = new VelocityRecord();
  public static Lidar lidar;// = new Lidar();

  public static AutoSwitches autoSwitches = new AutoSwitches();
  boolean manualStart = false;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    Compressor compressor = new Compressor(0);
    
    compressor.setClosedLoopControl(true);
    CameraServer.getInstance().startAutomaticCapture(); 
    
    oi = new OI();

    boolean pi = true;
    if (pi) {
      //client = new TCPClient();
      client = new UDPServer();
      client.start();
      targetInfo = client;
    } else {
      camera = new TargetCamera();
      camera.start();
      targetInfo = camera;
    }
    //System.out.println("Camera = " + ReadCameraPorts.readIRCamera());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //System.out.println(autoSwitches.getAutonCommand().getName());
    //velocityRecord.updateVelocity();
    //SmartDashboard.putNumber("lidar", lidar.getDistance());
    // SmartDashboard.putNumber("gyro", sensors.getHeading());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    //pneumatics.setState(pneumatics.RF_LATCH, false);
  }

  @Override
  public void disabledPeriodic() {
    //double[] driveEncoders = Robot.sensors.getDriveEncoders();
    //SmartDashboard.putNumber("left", driveEncoders[0]);
    //SmartDashboard.putNumber("right", driveEncoders[1]);
    
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    // RESTORE
    m_autonomousCommand = autoSwitches.getAutonCommand();
    manualStart = autoSwitches.getManualMode();
    System.out.println(manualStart + " " + m_autonomousCommand);

    //m_autonomousCommand = new DriveToJolt();

    sensors.resetGyro();
    sensors.resetDriveEncoders();
    sensors.resetPosition();
    sensors.resetPitch();
    
    intakeRotate.resetOffset();
    frontLift.resetEncoder();
    rearLift.resetEncoder();

    if (m_autonomousCommand != null && !manualStart) {
      gameState.setState(GameState.AUTON);
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    sensors.updatePosition();
    if (m_autonomousCommand != null && Robot.oi.driver.getBButton()) {
      if (manualStart && !m_autonomousCommand.isRunning()) {
        m_autonomousCommand.start();
        manualStart = false;
      }
    }
    if (m_autonomousCommand != null && Robot.oi.driver.getXButton()) {
      m_autonomousCommand.cancel();
      //m_autonomousCommand = null;
    }
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    //pneumatics.setState(pneumatics.SHIFT, false);
    
    /* RESTORE
    sensors.resetPosition();
    sensors.resetDriveEncoders();
    sensors.resetGyro();
    sensors.resetPitch();
    */
    if(Robot.driveTrain.isSwitched()) Robot.driveTrain.switchDirection();
    gameState.setState(GameState.TELEOP);
    if(m_autonomousCommand != null) m_autonomousCommand.cancel();
    
  }
  /**
   * This function is called periodically during operator control.
   */
  long lastTime = 0;
  double lastAverage = 0;
  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putNumber("Rear Lift Encoder", rearLift.getPosition());
    SmartDashboard.putNumber("Front Lift Encoder", frontLift.getPosition());
    /*
    double[] driveEncoders = sensors.getDriveEncoders();
  
    long time = System.currentTimeMillis();
    long elapsed = time - lastTime;
    double[] encoders = sensors.getDriveEncoders();
    System.out.println(encoders[0] + " " + encoders[1]);
    double average = (encoders[0] + encoders[1])/2.0;
    double distance = (average - lastAverage)/sensors.ENCODER_COUNTS_PER_INCH_LOW_GEAR;
    double speed = distance/elapsed * 1000;
    System.out.println("speed = " + speed);
    lastAverage = average;
    lastTime = time;
    double[] position = sensors.getPosition();
    System.out.println(position[0] + " " + position[1]);
    double[] targetInfo = client.getTargetInfo();
    System.out.println(targetInfo[0]-160 + " " + targetInfo[1] + " " + targetInfo[2]);
    */
    //double[] driveEncoders = sensors.getDriveEncoders();
    //System.out.println(driveEncoders[0] + " " + driveEncoders[1]);
    //System.out.println("pitch = " + sensors.getHeading());
    
    SmartDashboard.putNumber("Intake Rotate Encoder", sensors.getIntakeRotatePosition());
    /*
    SmartDashboard.putNumber("Left drive", driveEncoders[0]);
    SmartDashboard.putNumber("Right drive", driveEncoders[1]);
    SmartDashboard.putNumber("gyro", sensors.getHeading());
    
   
    SmartDashboard.putNumber("Front Lift State", RobotMap.mode);
    SmartDashboard.putNumber("Left drive", driveEncoders[0]);
    SmartDashboard.putNumber("Right drive", driveEncoders[1]);

    SmartDashboard.putNumber("Pitch", sensors.getPitch());
    SmartDashboard.putNumber("Front Lift Encoder", frontLift.getPosition());
    SmartDashboard.putNumber("Intake Rotate Encoder", sensors.getIntakeRotatePosition());
    SmartDashboard.putNumber("Rear Lift Encoder", rearLift.getPosition());
    */
    /*
    if (oi.driver.getXButton() && m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    */
    //SmartDashboard.putNumber("rearlift", Robot.rearLift.getPosition());
    Scheduler.getInstance().run();
    //sensors.updatePosition();
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
