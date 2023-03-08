// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TalonFX armMotor = new TalonFX(Constants.RoboRioPortConfig.ARM_MOTOR);
  private double speed = 0.6;
  private final double HIGH_POS = -177000;
  private final double HIGH_POS2 = -154000;
  private final double MID_POS = -140000;
  private final double CHASSIS_EXIT_POS = -130000;
  private double kP = 0.25;
  private static final int TIMEOUT = 30;
  private static final int SLOTIDX = 0;
  private static final int PIDLoopIdx = 0;

  private double cruiseVelocity = 20000;
  private double acceleration = 30000;
  private double closedLoopSpeed = 1.0;

 
  /** Creates a new Arm. */
  public Arm() {
    armMotor.configFactoryDefault();
    armMotor.clearStickyFaults();
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, SLOTIDX, TIMEOUT);
    armMotor.configNeutralDeadband(0.001, TIMEOUT);
    resetArmEncoder();

    /* Set relevant frame periods to be at least as fast as periodic rate */
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);


    /* Set the peak and nominal outputs */
		armMotor.configNominalOutputForward(0, TIMEOUT);
		armMotor.configNominalOutputReverse(0, TIMEOUT);
		armMotor.configPeakOutputForward(1, TIMEOUT);
		armMotor.configPeakOutputReverse(-1, TIMEOUT);

    armMotor.selectProfileSlot(SLOTIDX, PIDLoopIdx);
    armMotor.config_kP(0, kP, TIMEOUT);
    armMotor.config_kI(0,0, TIMEOUT);
    armMotor.config_kD(0,0, TIMEOUT);
    armMotor.config_kF(0,0, TIMEOUT);


    armMotor.configMotionCruiseVelocity(cruiseVelocity, TIMEOUT);
    armMotor.configMotionAcceleration(acceleration, TIMEOUT);
    armMotor.configClosedLoopPeakOutput(SLOTIDX, closedLoopSpeed);
    
    armMotor.configStatorCurrentLimit(
      new StatorCurrentLimitConfiguration(true, 30, 45, 0.050));

    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armUp(){
    armMotor.set(ControlMode.PercentOutput, -speed);
  }
  public void armDown(){
    armMotor.set(ControlMode.PercentOutput, speed);
  }
  public void stopArm(){
    armMotor.set(ControlMode.PercentOutput, 0);
  }
  public double getArmPosition(){
    return armMotor.getSelectedSensorPosition();
  }
  public void resetArmEncoder(){
    armMotor.setSelectedSensorPosition(0);
  }
  public void armMidGoal(){
    armMotor.set(ControlMode.MotionMagic, MID_POS);
  }
  public void armHighGoal(){
    armMotor.set(ControlMode.MotionMagic, HIGH_POS);
  } 

  public void armHighGoalMotionMagic(){
    armMotor.set(ControlMode.MotionMagic, HIGH_POS);
  }

  public void armHighGoal2(){
    armMotor.set(ControlMode.MotionMagic, HIGH_POS2);
  }
  public void armMoveToZeroPosition(){
    armMotor.set(ControlMode.MotionMagic, 0);
  }
  public boolean isAtMidGoal(){
    return getArmPosition()<=MID_POS+500;
  }
  public boolean isAtHighGoal(){
    return getArmPosition()<=HIGH_POS+500;
  }
  public boolean hasExitedChassis(){
    return getArmPosition()<= CHASSIS_EXIT_POS;
  }
  public boolean isAtHighGoal2(){
    return getArmPosition() >= HIGH_POS2-150;
  }
  public void slowMaxSpeed(){
    armMotor.configClosedLoopPeakOutput(0, 0.3);
  }
  public void restoreMaxSpeed(){
    armMotor.configClosedLoopPeakOutput(0, 0.5);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Speed (manual)", () -> speed, (value) -> this.speed = value);
    
    builder.addDoubleProperty("Speed (Closed Loop)", () -> closedLoopSpeed, (value) -> {
      this.closedLoopSpeed = value;
      armMotor.configClosedLoopPeakOutput(SLOTIDX, closedLoopSpeed);
    });

    builder.addDoubleProperty("Cruise Velocity", () -> cruiseVelocity, (value) -> {
      this.cruiseVelocity = value;
      armMotor.configMotionCruiseVelocity(cruiseVelocity, TIMEOUT);
    });
    
    builder.addDoubleProperty("Acceleration", () -> acceleration, (value) -> {
      this.acceleration = value;
      armMotor.configMotionAcceleration(acceleration, TIMEOUT);
    });
    
    builder.addDoubleProperty("Arm kP", () -> kP, (value) -> {
      kP = value;
      armMotor.config_kP(0, kP);
    });
  }
}
