/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_PID extends SubsystemBase {
  /**
   * Creates a new Shooter_PID.
   */
  public Shooter_PID() {

  }
  
  double xvalue = 0;
  double distanceperpulse = 0; 
  double fixpercentp = 0;
  private double integral = 0;
  private double derivative = 0;
  private double previous_error = 0;
  private double shooter_cmd;
  Encoder enc = new Encoder(1, 2);


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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
