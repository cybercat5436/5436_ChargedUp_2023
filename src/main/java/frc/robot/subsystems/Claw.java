// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.element.ModuleElement.DirectiveVisitor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Claw extends SubsystemBase {
  /** Creates a new ConeIntake. */
  //Instantiate Motors
  private CANSparkMax clawMotor = new CANSparkMax(Constants.RoboRioPortConfig.CLAW_MOTOR, MotorType.kBrushless);
  private double speed = 0.5;
  private RelativeEncoder clawEncoder = clawMotor.getEncoder();


  public Claw() {
    clawMotor.restoreFactoryDefaults();
    clawMotor.clearFaults();
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Claw Motor", clawMotor.get());
  }

  public void clawGrab(){
    clawMotor.set(speed);
  }
  public void clawRelease(){
    clawMotor.set(speed*-1);
  }
  public void stopGrab(){
    clawMotor.set(0);
  }
  public double getClawPosition(){
    return clawEncoder.getPosition();
  }
  public void resetClawEncoder(){
    clawEncoder.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Claw Speed", () -> speed, (value) -> speed = value);
    builder.addDoubleProperty("Claw Position", () -> getClawPosition(), null);
  }
}