/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Climb;

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
  //Subsystems
  private Intake Intake = new Intake();
  private Climb Climb = new Climb();

  //Basic PID Stuff
  private double integral = 0;
  private double derivative = 0;
  private double error;
  private double previous_error;
  private double shooter_cmd;
  private double Kp, Ki, Kd;
  private double desiredPosition;
 
 

  //limelight stuff
  private boolean limelight_has_target = false;
  private double limelight_drive_command;
  private double limelight_steer_command;
  private double steer_cmd;
  double STEER_Kd, STEER_Ki, STEER_Kp;

  //Drivetrain
  CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax left2 = new CANSparkMax(2, MotorType.kBrushless);
  SpeedControllerGroup left = new SpeedControllerGroup(left1, left2);
  CANSparkMax right1 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);
  CANEncoder left1enc = new CANEncoder(left1);
  CANEncoder right1enc = new CANEncoder(right1);
  SpeedControllerGroup right = new SpeedControllerGroup(right1, right2);
  DifferentialDrive drive = new DifferentialDrive(left, right);

  //Controllers
  XboxController xbox = new XboxController(0);
  Joystick panel = new Joystick(1);
  JoystickButton close = new JoystickButton(panel, 1);
  JoystickButton medium = new JoystickButton(panel, 2);
  JoystickButton far = new JoystickButton(panel, 3);
  JoystickButton telescopeUp = new JoystickButton(panel, 4);
  JoystickButton telescopeDown = new JoystickButton(panel, 5);
  JoystickButton climbUp = new JoystickButton(panel, 6);
  JoystickButton climbDown = new JoystickButton(panel, 7);
  JoystickButton aim = new JoystickButton(panel, 8);

  //Shooter
  WPI_TalonSRX shooter = new WPI_TalonSRX(8);
  private double shooterSpeed;
  private double shooterDesired;

  //compressor
  Compressor comp = new Compressor();

  //Pnuematics
  DoubleSolenoid gearswitch = new DoubleSolenoid(0, 1);

  //timer
  Timer timer = new Timer();

  //Navx
  AHRS navX = new AHRS(Port.kMXP);
  private double desiredAngle;
  private double navXIntegral, navXDerivative, navXPrevious_error, navX_steer_command, controlKp, controlKi, controlKd;

  //Encoders
  Encoder feedEncoder = new Encoder(0, 1);
  TalonSRXFeedbackDevice shooterEncoder; 

  double drivemode = 0;


   //Autonomous Stages
   public enum Autostage {
    kStart,
    kLimelight,
    kShootBall1,
    kWait1,
    kShootBall2,
    kWait2,
    kShootBall3,
    kDone;
  }

  private double stage;
  Autostage Currentstage = Autostage.kStart;
  


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    left1.setSmartCurrentLimit(40);
    left1.setSmartCurrentLimit(40);
    left1.setSmartCurrentLimit(40);
    left1.setSmartCurrentLimit(40);
    feedEncoder.reset();
    comp.setClosedLoopControl(false);
    shooter.configFactoryDefault(20);
    shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
    
  }


  public void Limelight_Steer_Tracking(){
    final double tx = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tx").getDouble(0);
    final double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    //Kp = 0.03;
    //Ki = 0.19354838709;
    //Kd = 0.0011625;
    Kp = 0.05;
    Ki = 0.06137254901;
    Kd = 0.0006;
    error = tx;

   
    
    this.previous_error = error;
  
    this.previous_error = (error - this.previous_error) / .02;
     
    if (tv < 1.0){
      limelight_has_target = false;
      limelight_drive_command = 0.0;
      return;
    }
  
    limelight_has_target = true;
    
    STEER_Kp = error * Kp;
    SmartDashboard.putNumber("Kp", STEER_Kp);
  
    this.integral += (error * .02);
    STEER_Ki = this.integral * Ki;
    SmartDashboard.putNumber("Ki", STEER_Ki);
  
    derivative = (error - this.previous_error) / .02;
    STEER_Kd = derivative * Kd;
    SmartDashboard.putNumber("Kd", STEER_Kd);
  
    limelight_steer_command = STEER_Kd + STEER_Ki + STEER_Kp; 
    SmartDashboard.putNumber("Steer", limelight_steer_command);

  }

  

  public void Limelight_Close_Tracking() {
    final double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    final double ta = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("ta").getDouble(0);
    final double Kp = 4;
    final double DESIRED_TARGET_AREA = .5;
    final double error = DESIRED_TARGET_AREA - ta;

    if (tv < 1.0) {
      limelight_has_target = false;
      limelight_drive_command = 0.0;
      return;
    }

    limelight_has_target = true;

    final double DRIVE_Kp = error * Kp;

    final double drive_cmd = DRIVE_Kp;
    limelight_drive_command = drive_cmd;

  }

  public void Limelight_Medium_Tracking() {
    final double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    final double ta = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("ta").getDouble(0);
    final double Kp = 4;
    final double DESIRED_TARGET_AREA = 1;
    final double error = DESIRED_TARGET_AREA - ta;

    if (tv < 1.0) {
      limelight_has_target = false;
      limelight_drive_command = 0.0;
      return;
    }

    limelight_has_target = true;

    final double DRIVE_Kp = error * Kp;

    final double drive_cmd = DRIVE_Kp;
    limelight_drive_command = drive_cmd;

  }

  public void Limelight_Far_Tracking() {
    final double tv = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tv").getDouble(0);
    final double ta = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("ta").getDouble(0);
    final double Kp = 4;
    final double DESIRED_TARGET_AREA = 1.5;
    final double error = DESIRED_TARGET_AREA - ta;

    if (tv < 1.0) {
      limelight_has_target = false;
      limelight_drive_command = 0.0;
      return;
    }

    limelight_has_target = true;

    final double DRIVE_Kp = error * Kp;

    final double drive_cmd = DRIVE_Kp;
    limelight_drive_command = drive_cmd;

  }

  public void ShooterPID(){
    Kp = 30;
    double currentRate = shooter.getSelectedSensorVelocity();
    shooterDesired = shooterSpeed * 460;
    error = shooterDesired - currentRate;

    final double ShooterRateKp = error * Kp;

    shooter_cmd = ShooterRateKp / 460;
    
  } 

  public void NavXPID(){
    Kp = 0.04;
    Ki = 0.00237254901;
    Kd = 0.0001;
    error = desiredAngle - navX.getAngle();

    this.navXPrevious_error = error;
  
    this.navXPrevious_error = (error - this.navXPrevious_error) / .02;
     
    
    controlKp = error * Kp;
    SmartDashboard.putNumber("navXKp", controlKp);

    this.navXIntegral += (error * .02);
    controlKi = this.navXIntegral * Ki;
    SmartDashboard.putNumber("navXKi", controlKi);
  
    navXDerivative = (error - this.previous_error) / .02;
    controlKd = navXDerivative * Kd;
    SmartDashboard.putNumber("navXKd", controlKd);
  
    navX_steer_command = controlKp + controlKi + controlKd;
    SmartDashboard.putNumber("navXcontrol", navX_steer_command);
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
    SmartDashboard.putNumber("FeederEncoder", feedEncoder.getRate());
   
    SmartDashboard.putNumber("NavX", navX.getAngle());

    SmartDashboard.putNumber("ShooterEncoder", shooter.getSelectedSensorVelocity());

    SmartDashboard.putNumber("left1", left1enc.getPosition());

    SmartDashboard.putNumber("right1", right1enc.getPosition());

    comp.setClosedLoopControl(false);

   
    
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
  @Override
  public void autonomousInit() {
    Currentstage = Autostage.kStart;
    left1enc.setPosition(0);
    right1enc.setPosition(0);
    navX.reset();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    ShooterPID();
    NavXPID();
    final double tx = NetworkTableInstance.getDefault().getTable("limelight-bison").getEntry("tx").getDouble(0);
    Limelight_Steer_Tracking();

    if (Currentstage == Autostage.kStart){
      drive.arcadeDrive(.7, 0);
      shooter.set(shooter_cmd);
      shooterSpeed = .88;
      if (right1enc.getPosition() <= -30 && left1enc.getPosition() >= 30){
        Currentstage = Autostage.kLimelight;
      }
    }

  if (Currentstage == Autostage.kLimelight){
    drive.arcadeDrive(0, -limelight_steer_command*.5);
    drive.setMaxOutput(0.2);
    if (tx >= -.6 && tx <= .6){
      Currentstage = Autostage.kShootBall1;
    }
  }

  if (Currentstage == Autostage.kShootBall1){
    shooterSpeed =.9;
    shooter.set(shooter_cmd);
    stage = 1;
    if (shooter.getSelectedSensorVelocity() > 330 && shooter.getSelectedSensorVelocity() < 500){
      Intake.feedShooter();
    }

    else {
      Intake.StopFeeder();
    }

    if (shooter.getSelectedSensorVelocity() < 300){
      Currentstage = Autostage.kWait1;
    }
  }

  if (Currentstage == Autostage.kWait1){
    shooterSpeed =.88;
    shooter.set(shooter_cmd);
    Intake.MoveIn();
    Intake.StopFeeder();

    stage = 2;
    if (shooter.getSelectedSensorVelocity() > 330 && shooter.getSelectedSensorVelocity() < 500){
      Currentstage = Autostage.kShootBall2;
    }
  }

  if (Currentstage == Autostage.kShootBall2){
    shooterSpeed =.88;
    shooter.set(shooter_cmd);
    Intake.StopMover();
    stage = 3;
    Intake.feedShooter();
    

    if (shooter.getSelectedSensorVelocity() < 300){
      Currentstage = Autostage.kWait2;
    }
  }

  if (Currentstage == Autostage.kWait2){
    shooterSpeed =.88;
    shooter.set(shooter_cmd);
    stage = 4;
    Intake.StopFeeder();
    Intake.MoveIn();


    if (shooter.getSelectedSensorVelocity() > 330 && shooter.getSelectedSensorVelocity() < 500){
      Currentstage = Autostage.kShootBall3;
    }
  }

  if (Currentstage == Autostage.kShootBall3){
    shooterSpeed =.88;
    shooter.set(shooter_cmd);
    stage = 5;
    Intake.feedShooter();


    if (shooter.getSelectedSensorVelocity() < 300){
      Currentstage = Autostage.kDone;
    }

  }

  if (Currentstage == Autostage.kDone){
    drive.arcadeDrive(0, 0);
    shooter.set(0);
  }

  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    ShooterPID();

    if (telescopeUp.get()){
      Climb.telescopeUp();
    }

    else if (telescopeDown.get()){
      Climb.telescopeDown();
    }

    else {
      Climb.stopTelescope();
    }

    if(xbox.getPOV() == 0){
      if (climbUp.get()){
        Climb.climbUp();
      }

      else if (climbDown.get()){
        Climb.climbDown();
      }
    
      else {
        Climb.stopClimb();
      }
  }

  else {
    Climb.stopClimb();
  }
    
    if (far.get()){
      Limelight_Far_Tracking();
      Limelight_Steer_Tracking();
      gearswitch.set(DoubleSolenoid.Value.kReverse);
      drive.setMaxOutput(.3);
      shooterSpeed = 0.8;

      if (limelight_has_target){
        drive.arcadeDrive( -limelight_drive_command * .6, -limelight_steer_command*.7);
      }
  
      else
      {
        drive.arcadeDrive(0, 0);
      }
    }

      else if (medium.get()){
        Limelight_Medium_Tracking();
        Limelight_Steer_Tracking();
        gearswitch.set(DoubleSolenoid.Value.kReverse);
        drive.setMaxOutput(.3);
        shooterSpeed = 0.88;


        if (limelight_has_target){
          drive.arcadeDrive( -limelight_drive_command * .6, -limelight_steer_command*.7);
        }
  
      else
      {
        drive.arcadeDrive(0, 0);
      }
    }

    else if (close.get()){
      Limelight_Close_Tracking();
      Limelight_Steer_Tracking();
      gearswitch.set(DoubleSolenoid.Value.kReverse);
      drive.setMaxOutput(.3);
      shooterSpeed = 1;

      if (limelight_has_target){
        drive.arcadeDrive( -limelight_drive_command * .6, -limelight_steer_command*.7);
      }

      else
      {
        drive.arcadeDrive(0, 0);
      }

    }

    else if (aim.get()){
      Limelight_Steer_Tracking();
      gearswitch.set(DoubleSolenoid.Value.kReverse);
      drive.setMaxOutput(.3);

      if (limelight_has_target){
        drive.arcadeDrive( 0 , -limelight_steer_command);
      }

      else
      {
        drive.arcadeDrive(0, 0);
      }

    }

    else {
      drive.setMaxOutput(1);
      if (xbox.getStickButton(Hand.kRight)){
        gearswitch.set(DoubleSolenoid.Value.kForward);
        drivemode = 1;
      }
  
      else if (xbox.getStickButton(Hand.kLeft)){
        gearswitch.set(DoubleSolenoid.Value.kReverse);
        drivemode = 0;
      }

      if (drivemode == 0){
        drive.arcadeDrive(xbox.getRawAxis(1)*.9, -xbox.getRawAxis(4)*.75);
      }

      else {
        drive.curvatureDrive(xbox.getRawAxis(1)*.8, -xbox.getRawAxis(4), false);
      }
    }

    if (xbox.getXButton()){
      Intake.MoveIn();
    }

    else if (xbox.getBButton()){
      Intake.MoveOut();
    }

    else if (xbox.getBumper(Hand.kLeft)){
      Intake.Center();
    }

    else if (xbox.getTriggerAxis(Hand.kLeft) >= 0.8){
      Intake.PickUp();
    }

    else if (xbox.getAButton()){
      Intake.feedShooter();
    }

    else if (xbox.getPOV() == 180){
      Intake.unFeedShooter();
    }

    else if (xbox.getYButton()){
      Intake.SpitOut();
    }

    else {
      Intake.StopMover();
      Intake.StopIntake();
      Intake.StopFeeder();
    }

    if (xbox.getTriggerAxis(Hand.kRight) >= 0.8){
      shooter.set(shooter_cmd);
    }

    else {
      shooter.set(0);
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
