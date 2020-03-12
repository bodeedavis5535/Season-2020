/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  CANSparkMax climb1 = new CANSparkMax(10, MotorType.kBrushless);
  CANSparkMax climb2 = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax climb3 = new CANSparkMax(12, MotorType.kBrushless);

  WPI_VictorSPX telescope = new WPI_VictorSPX(14);
  
  public void telescopeUp(){
    telescope.set(1);
  } 

  public void telescopeDown(){
    telescope.set(-1);
  }

  public void stopTelescope(){
    telescope.set(0);
  }

  public void climbUp(){
    climb1.set(0.5);
    climb2.set(0.5);
    climb3.set(0.5);
  }

  public void climbDown(){
    climb1.set(-0.5);
    climb2.set(-0.5);
    climb3.set(-0.5);
  }

  public void stopClimb(){
    climb1.set(0);
    climb2.set(0);
    climb3.set(0);
  }
}
