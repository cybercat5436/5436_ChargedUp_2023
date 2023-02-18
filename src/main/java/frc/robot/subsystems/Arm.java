// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TalonFX armMotor = new TalonFX(Constants.RoboRioPortConfig.ARM_MOTOR);
  private double speed = 0.2;
  private final double HIGH_POS = -49500;
  private final double MID_POS = -39000;
  private double kP = 0.25;
 
  /** Creates a new Arm. */
  public Arm() {
    armMotor.configFactoryDefault();
    armMotor.clearStickyFaults();
    armMotor.setNeutralMode(NeutralMode.Brake);
    resetArmEncoder();
    armMotor.config_kP(0, kP);
    armMotor.configClosedLoopPeakOutput(0, 0.2);
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armUp(){
    armMotor.set(ControlMode.PercentOutput, speed);
  }
  public void armDown(){
    armMotor.set(ControlMode.PercentOutput, speed*-1);
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
    armMotor.set(ControlMode.Position, MID_POS);
  }
  public void armHighGoal(){
    armMotor.set(ControlMode.Position, HIGH_POS);
  } 
  public void armMoveToZeroPosition(){
    armMotor.set(ControlMode.Position, 0);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Arm Speed", () -> speed, (value) -> speed = value);
    builder.addDoubleProperty("Arm Position", () -> getArmPosition(), null);
    builder.addDoubleProperty("Arm kP", () -> kP, (value) -> {
      kP = value;
      armMotor.config_kP(0, kP);
    });
  }
}
