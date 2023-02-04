// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  /** Creates a new ConeIntake. */
  //Instantiate Motors
  private CANSparkMax grabber = new CANSparkMax(31, MotorType.kBrushed);
  

  public Grabber() {
    grabber.restoreFactoryDefaults();
    grabber.clearFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Grabber Motor", grabber.get());
  }

  public void grab(double speed, String direction){
    if(direction.equals("detract")){
      speed *= -1;
    }
    grabber.set(speed);
  }

  public void stopGrab(){
    grabber.set(0);
  }
}