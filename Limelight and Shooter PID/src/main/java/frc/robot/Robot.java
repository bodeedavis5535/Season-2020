/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter_PID;
import edu.wpi.first.wpilibj.DriverStation;
 
/**
* The VM is configured to automatically run this class, and to call the
* functions corresponding to each mode, as described in the TimedRobot
* documentation. If you change the name of this class or the package after
* creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
 private static final String kDefaultAuto = "Default";
 private static final String kCustomAuto = "My Auto";
 private String m_autoSelected;
 private final SendableChooser<String> m_chooser = new SendableChooser<>();


 //controllers
 XboxController xbox;
 Joystick panel = new Joystick(1);
 JoystickButton shootMode = new JoystickButton(panel, 1);
 JoystickButton intakeMode = new JoystickButton(panel, 2);
 JoystickButton driveMode = new JoystickButton(panel, 3);
 JoystickButton limelightAlignmnet = new JoystickButton(panel, 4);

 //drivetrain
 CANSparkMax left1, left2, right1, right2;
 SpeedControllerGroup left, right;
 DifferentialDrive m_drive;
 final double steer = xbox.getX(Hand.kRight);
 final double drive = -xbox.getY(Hand.kLeft);

 //Subsystems
 private final Shooter_PID Shooter_PID = new Shooter_PID();
 private final Intake Intake = new Intake();

  // limelight
  private boolean limelight_has_target = false;
  private double limelight_drive_command = 0.0;
  private double limelight_steer_command = 0.0;

  // Shooter
  WPI_TalonSRX shooter_motor = new WPI_TalonSRX(5);
  Encoder enc = new Encoder(1, 2);
  double xvalue = 0;
  double distanceperpulse = 0;
  double fixpercentp = 0;

  //general PID
  private double integral = 0;
  private double derivative = 0;
  private final double previous_error = 0;
  private double shooter_cmd;
  
  //Driverstation
  static DriverStation DriverStation;

  // Colors
  final char red = 'R';
  final char green = 'G';
  final char blue = 'B';
  final char yellow = 'Y';

  //Autonomous SetUp
  public enum Autonomous{
    kAim,
    kshoot3,
    kback,
    kturn1,
    kengagevision2,
    kballpickup,
    kturn2,
    kengagevision3,
    kshoot5;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    left1 = new CANSparkMax(1, MotorType.kBrushless);
    left2 = new CANSparkMax(2, MotorType.kBrushless);
    left = new SpeedControllerGroup(left1, left2);
    right1 = new CANSparkMax(3, MotorType.kBrushless);
    right2 = new CANSparkMax(4, MotorType.kBrushless);
    right = new SpeedControllerGroup(right1, right2);
    m_drive = new DifferentialDrive(left, right);

    xbox = new XboxController(0);

    shooter_motor = new WPI_TalonSRX(5);

    String gameData;
    gameData = edu.wpi.first.wpilibj.DriverStation.getInstance().getGameSpecificMessage();

    SmartDashboard.putString("color", gameData);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */

   /*
  public void Limelight_Steer_Tracking() {
    final double tx = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tx").getDouble(0);
    final double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    final double Kp = 0.4;
    final double Ki = 0.25;
    final double Kd = 0.1;
    final double error = tx;

    if (tv < 1.0) {
      limelight_has_target = false;
      limelight_steer_command = 0.0;
      return;
    }

    limelight_has_target = true;

    final double STEER_Kp = Kp * error;
    this.integral += error * .05;
    derivative = (error - this.previous_error) / .05;

    final double steer_cmd = STEER_Kp + Ki * this.integral + Kd * derivative;

    limelight_steer_command = steer_cmd;

  }

  public void Limelight_Drive_Tracking() {
    final double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    final double ta = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("ta").getDouble(0);
    final double Kp = 0.5;
    final double Ki = 0.25;
    final double Kd = 0.1;
    final double DESIRED_TARGET_AREA = 30.0;
    final double error = DESIRED_TARGET_AREA - ta;

    if (tv < 1.0) {
      limelight_has_target = false;
      limelight_drive_command = 0.0;
      return;
    }

    limelight_has_target = true;

    final double DRIVE_Kp = error * Kp;
    this.integral += error * 0.05;
    derivative = (error - this.previous_error) / .05;
    final double drive_cmd = this.integral * Ki + DRIVE_Kp + Kd * derivative;
    limelight_drive_command = drive_cmd;
  }*/

  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    /*
    Limelight_Drive_Tracking();
    Limelight_Steer_Tracking();
    Shooter_PID.Shooter_PID();

    
   if (limelightAlignmnet.get()){
     if (limelight_has_target){
       m_drive.arcadeDrive( limelight_drive_command, limelight_steer_command);
     }
 
     else
     {
       m_drive.arcadeDrive(0, 0);
     }

   }

 else {
   m_drive.arcadeDrive(drive, steer);
 }
 */

m_drive.curvatureDrive(-xbox.getRawAxis(1), xbox.getRawAxis(4), true);

//manual shoot
 if (xbox.getYButton()){
  shooter_motor.set(1);
 }

 else {
   shooter_motor.set(0);
 }

 if (xbox.getAButton()){
   Intake.ballIn();
 }

 else if (xbox.getBButton()){
   Intake.ballOut();
 }

 else {
   Intake.ballStationary();
 }


 /*
 //These are the 3 major drive modes.
 if (intakeMode.get()){
   Intake.intakeDown();
   Intake.activate();
 }

 if (shootMode.get()){
  Intake.deactivate();
  Intake.intakeUp();
  shooter_motor.set(shooter_cmd);
 }
 
 if (driveMode.get()){
  Intake.deactivate();
  Intake.intakeUp();
 }
 */

 }
 
 /**
  * This function is called periodically during test mode.
  */
 @Override
 public void testPeriodic() {
 }
}
 

