// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extender extends SubsystemBase {
  private CANSparkMax extenderMotor = new CANSparkMax(Constants.RoboRioPortConfig.EXTENDER_MOTOR, MotorType.kBrushless);
  private RelativeEncoder extenderEncoder = extenderMotor.getEncoder();
  private double speed = 0.6;
  private SparkMaxPIDController extenderPID = extenderMotor.getPIDController();
  private double kP = 0.1;
  private double desiredMidGoal = 96.17;
  private DigitalInput zeroLimitSwitch = new DigitalInput(Constants.RoboRioPortConfig.EXTENDER_ZERO_LIMIT_SWITCH);
  private DigitalInput maxLimitSwitch = new DigitalInput(Constants.RoboRioPortConfig.EXTENDER_MAX_LIMIT_SWITCH);
  //changed
  private double desiredHighGoal = 244.6; // NOT TESTED
  private double retractLimit = 5.56;

  /** Creates a new Extender. */
  public Extender() {
    extenderMotor.restoreFactoryDefaults();
    extenderMotor.clearFaults();
    extenderMotor.setSmartCurrentLimit(30, 30);

    extenderMotor.setIdleMode(IdleMode.kBrake);
    extenderMotor.setInverted(true);
    extenderPID.setP(kP);
    extenderPID.setOutputRange(-1, 1);
    resetExtenderEncoder();
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
    if(maxLimitSwitch.get()){
      extenderMotor.set(speed);
    }
  }
  public void retract(){
    if(zeroLimitSwitch.get()){
      extenderMotor.set(speed*-1);
    }else{
      resetExtenderEncoder();
    }
  }
  public boolean maxLimitSwitch(){
    return maxLimitSwitch.get();
  }
  public boolean zeroLimitSwitch(){
    return zeroLimitSwitch.get();
  }
  public double getExtenderPosition(){
    return extenderEncoder.getPosition();
  }
  public void resetExtenderEncoder(){
    extenderEncoder.setPosition(0);
  }
  public void gotoDefaultPos(){
    extenderPID.setReference(0, CANSparkMax.ControlType.kPosition);
  }
  public void extendMidGoal(){
    extenderPID.setReference(desiredMidGoal, CANSparkMax.ControlType.kPosition);
  }
  public void extendHighGoal(){
    extenderPID.setReference(desiredHighGoal, CANSparkMax.ControlType.kPosition);
  }
  public boolean isRetracted(){
    return extenderEncoder.getPosition()< retractLimit;
  }
  public boolean isAtHighGoal(){
    return extenderEncoder.getPosition()>=desiredHighGoal-50;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Extender Speed", () -> speed, (value) -> speed = value);
    builder.addDoubleProperty("Extender Position", () -> getExtenderPosition(), null);
    builder.addDoubleProperty("Extender kP", () -> kP, (value) ->{
      kP = value;
      extenderPID.setP(kP);
    });
    builder.addDoubleProperty("Midgoal Desired Rotations", () -> desiredMidGoal, (value)->desiredMidGoal=value);
    builder.addDoubleProperty("Highgoal Desired Rotations", () -> desiredHighGoal, (value)->desiredHighGoal=value);

    builder.addBooleanProperty("Zero Limit Switch", () -> zeroLimitSwitch.get(), null);
  }
}
