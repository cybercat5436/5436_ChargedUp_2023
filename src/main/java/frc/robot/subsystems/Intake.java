// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.RoboRioPortConfig.INTAKE_MOTOR, MotorType.kBrushless);
  private double speed = 0.5;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.clearFaults();
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Intake Speed", () -> speed, (value) -> speed = value);

  }

  public void stopIntake(){
    intakeMotor.set(0);
  }
  public void intakeFeedIn(){
    intakeMotor.set(speed);
  }
  public void intakeFeedOut(){
    intakeMotor.set(speed*-1);
  }
}
