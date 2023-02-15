// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extender extends SubsystemBase {
  private CANSparkMax extenderMotor = new CANSparkMax(Constants.RoboRioPortConfig.EXTENDER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder extenderEncoder = extenderMotor.getEncoder();
  private double speed = 0.5;
  
  /** Creates a new Extender. */
  public Extender() {
    extenderMotor.restoreFactoryDefaults();
    extenderMotor.clearFaults();
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopExtend(){
    extenderMotor.set(0);
  }
  public void extend(){
    extenderMotor.set(speed);
  }
  public void retract(){
    extenderMotor.set(speed*-1);
  }
  public double getExtenderPosition(){
    return extenderEncoder.getPosition();
  }
  public void resetExtenderEncoder(){
    extenderEncoder.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Extender Speed", () -> speed, (value) -> speed = value);
    builder.addDoubleProperty("Extender Position", () -> getExtenderPosition(), null);
  }
}
