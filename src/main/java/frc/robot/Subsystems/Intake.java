/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  //intake
  WPI_VictorSPX ballPickup1 = new WPI_VictorSPX(5);
  WPI_VictorSPX ballPickup2 = new WPI_VictorSPX(9);

  //Movement of balls through the robot
  WPI_VictorSPX ballIntake1 = new WPI_VictorSPX(6);
  WPI_VictorSPX ballIntake2 = new WPI_VictorSPX(7);
  WPI_VictorSPX feeder = new WPI_VictorSPX(13);

  
  public Intake() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Center(){
    ballPickup1.set(.2);
    ballPickup2.set(-.2);
  }

  public void PickUp(){
    ballPickup1.set(.5);
    ballPickup2.set(-.5);

    ballIntake1.set(.25);
    ballIntake2.set(-.25);

    feeder.set(-.3);
  }

  public void AutonPickUp(){
    ballPickup1.set(.4);
    ballPickup2.set(-.4);

    ballIntake1.set(.3);
    ballIntake2.set(-.3);

    feeder.set(-.3);
  }

  public void SpitOut(){
    ballPickup1.set(-.45);
    ballPickup2.set(.45);
  }

  public void StopIntake(){
    ballPickup1.set(0);
    ballPickup2.set(0);
  }

  public void MoveIn(){
    ballIntake1.set(.4);
    ballIntake2.set(-.4);
  }

  public void MoveOut(){
    ballIntake1.set(-.4);
    ballIntake2.set(.4);
  }

  public void StopMover(){
    ballIntake1.set(0);
    ballIntake2.set(0);
  }

  public void feedShooter(){
    feeder.set(1);
  }

  public void StopFeeder(){
    feeder.set(0);
  }

  public void unFeedShooter(){
    feeder.set(-1);
  }



}
