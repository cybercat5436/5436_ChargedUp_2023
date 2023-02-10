// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Orienter extends SubsystemBase {
  private CANSparkMax orienterMotor = new CANSparkMax(4, MotorType.kBrushless);
  public LimeLight2 limelight2;




  /** Creates a new Orienter. */
  public Orienter(LimeLight2 limeLight2) {
    this.limelight2 = limeLight2;

  }

  public void microwaveSpin() {
    if(!limelight2.isOriented()) {
      orienterMotor.set(.1);
    } else{
      orienterMotor.set(0);
    }
  }
  public void stopMicrowave() {
    orienterMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
