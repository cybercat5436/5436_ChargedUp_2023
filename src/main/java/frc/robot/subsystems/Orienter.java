// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Orienter extends SubsystemBase implements Sendable{
  private CANSparkMax orienterMotor = new CANSparkMax(Constants.RoboRioPortConfig.ORIENT_MOTOR, MotorType.kBrushless);
  public LimeLight limelightOrient;
  private double spinKConstant = .05;
  private double spinOtherValue = 75;




  /** Creates a new Orienter. */
  public Orienter(LimeLight limeLight) {
    orienterMotor.clearFaults();
    orienterMotor.restoreFactoryDefaults();
    orienterMotor.setSmartCurrentLimit(30, 30);
    orienterMotor.setIdleMode(IdleMode.kBrake);
    this.limelightOrient = limeLight;
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
        SmartDashboard.putData(this);
  }

  public void microwaveSpin() {
    if(!limelightOrient.isOriented()) {
      orienterMotor.set(.1* spinKConstant * (spinOtherValue - limelightOrient.tLongLocal.getDouble(spinKConstant)));

    } else{
      orienterMotor.set(0);
    }
  }
  public void microwaveManualSpin(){
    orienterMotor.set(0.25);
  }
  public void microwaveReverseManualSpin(){
    orienterMotor.set(-0.25);
  }
  public void stopMicrowave() {
    orienterMotor.set(0);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor speed", orienterMotor.get());
    // This method will be called once per scheduler run
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("spinThreshold", () -> limelightOrient.spinThreshold, (value) -> limelightOrient.spinThreshold = value);
    builder.addDoubleProperty("spinKConstant", () -> spinKConstant, (value) -> spinKConstant = value);
    builder.addDoubleProperty("OtherValue", () -> spinOtherValue, (value) -> spinOtherValue = value);

}
}
