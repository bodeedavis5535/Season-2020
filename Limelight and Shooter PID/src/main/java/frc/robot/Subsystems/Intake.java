/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax intakeControl = new CANSparkMax(6, MotorType.kBrushless);
  CANEncoder placementEncoder = new CANEncoder(intakeControl);
  double currentPlacement = placementEncoder.getPosition();
  double desiredPlacement;
  double error = desiredPlacement - currentPlacement;
  final double kP = 0.5;
  private double action;
  final double maxDistance = 0;
  double motorpower = action / maxDistance;
  WPI_TalonSRX intakePower = new WPI_TalonSRX(6);
  WPI_VictorSPX ballMovement1 = new WPI_VictorSPX(7);
  WPI_VictorSPX ballMovement2 = new WPI_VictorSPX(8);
  
  public void activate(){
    intakePower.set(1);
  }

  public void deactivate(){
    intakePower.set(ControlMode.PercentOutput, 0);
  }

  public void intakeUp(){
    desiredPlacement = 0;
    action = error * kP;
    //measure what the total distance on the max form
    intakeControl.set(motorpower);
  }

  public void intakeDown(){
    desiredPlacement = maxDistance;
    action = error * kP;
    intakeControl.set(motorpower);
  }

  public void ballIn(){
    ballMovement1.set(1);
    ballMovement2.set(-1);
  }

  public void ballOut(){
    ballMovement1.set(-1);
    ballMovement2.set(1);
  }

  public void ballStationary(){
    ballMovement1.set(0);
    ballMovement2.set(0);
  }


}
