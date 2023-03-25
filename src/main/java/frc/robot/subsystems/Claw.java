// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.element.ModuleElement.DirectiveVisitor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
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
  private SparkMaxPIDController clawPID = clawMotor.getPIDController();
  private double speed = 0.5;
  private RelativeEncoder clawEncoder = clawMotor.getEncoder();
  private double kP = 0.1;
  private double cubeDesiredPos = 15;
  private double coneDesiredPos = 95;

  public Claw() {
    clawMotor.restoreFactoryDefaults();
    clawMotor.clearFaults();
    clawMotor.setSmartCurrentLimit(30, 30);
    clawMotor.setIdleMode(IdleMode.kBrake);
    clawMotor.setInverted(true);
    clawPID.setP(kP);
    clawPID.setOutputRange(-1, 1);
    resetClawEncoder();
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Claw Motor", clawMotor.get());
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
  public void grabCone(){
    clawPID.setReference(coneDesiredPos, CANSparkMax.ControlType.kPosition);
  }
  public void grabCube(){
    clawPID.setReference(cubeDesiredPos, CANSparkMax.ControlType.kPosition);
  }
  public void gotoDefaultPos(){
    clawPID.setReference(0, CANSparkMax.ControlType.kPosition);
  }
  public boolean isConeGrabbed(){
    return clawEncoder.getPosition()>=coneDesiredPos-4;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    // builder.addDoubleProperty("Claw Speed", () -> speed, (value) -> speed = value);
    builder.addDoubleProperty("Claw Position", () -> getClawPosition(), null);
    // builder.addDoubleProperty("Claw kP", () -> kP, (value) ->{
    //   kP = value;
    //   clawPID.setP(kP);
    // });
    // builder.addDoubleProperty("Cube Desired Rotations", () -> cubeDesiredPos, (value)->cubeDesiredPos=value);
    // builder.addDoubleProperty("Cone Desired Rotations", () -> coneDesiredPos, (value)->coneDesiredPos=value);
  }
}