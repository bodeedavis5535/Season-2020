/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot;
 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
 
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
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
 WPI_VictorSPX left1, left2, right1, right2;
 XboxController xbox;
 SpeedControllerGroup left, right;
 DifferentialDrive m_drive;
 NetworkTableInstance table;
 
 //limelight
 private boolean limelight_has_target = false;
 private double limelight_drive_command = 0.0;
 private double limelight_steer_command = 0.0;
 
 //Shooter
 WPI_TalonSRX shooter_motor = new WPI_TalonSRX(5);
 Encoder enc = new Encoder(1, 2);
 double xvalue = 0;
 double distanceperpulse = 0; 
 double fixpercentp = 0;
 private double integral = 0;
 private double derivative = 0;
 private double previous_error = 0;
 private double shooter_cmd;
 
 /**
  * This function is run when the robot is first started up and should be
  * used for any initialization code.
  */
 @Override
 public void robotInit() {
   m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
   m_chooser.addOption("My Auto", kCustomAuto);
   SmartDashboard.putData("Auto choices", m_chooser);
   left1 = new WPI_VictorSPX(1);
   left2 = new WPI_VictorSPX(2);
   left = new SpeedControllerGroup(left1, left2);
   right1 = new WPI_VictorSPX(3);
   right2 = new WPI_VictorSPX(4);
   right = new SpeedControllerGroup(right1, right2);
   m_drive = new DifferentialDrive(left, right);
  
   xbox = new XboxController(0);
 
   shooter_motor = new WPI_TalonSRX(5);
 
 
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
 }
 
 /**
  * This autonomous (along with the chooser code above) shows how to select
  * between different autonomous modes using the dashboard. The sendable
  * chooser code works with the Java SmartDashboard. If you prefer the
  * LabVIEW Dashboard, remove all of the chooser code and uncomment the
  * getString line to get the auto name from the text box below the Gyro
  *
  * <p>You can add additional auto modes by adding additional comparisons to
  * the switch structure below with additional strings. If using the
  * SendableChooser make sure to add them to the chooser code above as well.
  */
 
  public void Limelight_Steer_Tracking(){ 
    double tx = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tx").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    final double Kp = 0.4;
    final double Ki = 0.25;
    final double Kd = 0.1;
    double error = tx;

    if (tv < 1.0) {
      limelight_has_target = false;
      limelight_drive_command = 0.0;
      return;
     }

     limelight_has_target = true;

     double STEER_Kp = Kp * error;
     this.integral += error * .05;
     derivative = (error - this.previous_error) / .05;


    double steer_cmd = STEER_Kp + Ki*this.integral + Kd*derivative;

    limelight_steer_command = steer_cmd;

  }

  public void Limelight_Drive_Tracking(){
    double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("ta").getDouble(0);
    final double Kp = 0.5;
    final double Ki = 0.25;
    final double Kd = 0.1;
    double DESIRED_TARGET_AREA = 30.0;
    double error = DESIRED_TARGET_AREA - ta;

    if (tv < 1.0) {
      limelight_has_target = false;
      limelight_drive_command = 0.0;
      return;
     }

     limelight_has_target = true;


    double DRIVE_Kp = error * Kp;
    this.integral += error * 0.05;
    derivative = (error - this.previous_error) / .05;
    double drive_cmd = this.integral*Ki + DRIVE_Kp + Kd*derivative;
    limelight_drive_command = drive_cmd;
  }

  public void Shooter_PID(){
    double Kp = 0.5;
    double Ki = 0.25;
    double Kd = 0.1;
    double current_rate = enc.getRate();
    double desired_rate = 5000;
    double error = desired_rate - current_rate;
    double shooter_Kp = 0;
    double shooter_ratecmd;
    double max_rate = 5330;
    
    if (desired_rate != current_rate){
      shooter_Kp = error * Kp;
      this.integral = error * 0.05;
      derivative = (error - this.previous_error) / .05;
      shooter_ratecmd = shooter_Kp + this.integral*Ki + Kd*derivative;
      shooter_cmd = shooter_ratecmd / max_rate;
    }

  }

  /*
  public void PID(){
   double ta = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("ta").getDouble(0);
   if (ta < 5){
     xvalue = 2500;
   }
 
   if (ta > 20){
     xvalue = 5000;
   }
 
  
   double kp = .5;
   double ki = .5;
   double kd = .5;
   double rate = enc.getRate();
   double desired_rate = xvalue;
   double error = desired_rate - rate;
   double max_rate = 5330;
   double current_percentage = rate/max_rate;
 
   distanceperpulse = enc.getDistancePerPulse();
   /* make something here that you divide the distance per pulse by to get rmp*
 
   if (rate != desired_rate ){
     double rateP = error * kp;
     double percentP = rateP / max_rate;
     fixpercentp = percentP + current_percentage;
     shooter_motor.set(fixpercentp);
    
   }
 
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
 
   Limelight_Drive_Tracking();
   Limelight_Steer_Tracking();
   Shooter_PID();
 
   double steer = xbox.getX(Hand.kRight);
   double drive = -xbox.getY(Hand.kLeft);
   boolean auto = xbox.getAButton();
 
 
   if (auto){
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
 

 if (xbox.getYButton()){
  shooter_motor.set(shooter_cmd);
 }
 
 
 }
 
 /**
  * This function is called periodically during test mode.
  */
 @Override
 public void testPeriodic() {
 }
}
 

