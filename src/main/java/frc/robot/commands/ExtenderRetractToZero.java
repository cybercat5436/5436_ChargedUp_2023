// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;
public class ExtenderRetractToZero extends CommandBase {
  /** Creates a new extenderRetractToZero. */
  private Extender extender;
  private Timer timer;
  private double timeLimit;
  public ExtenderRetractToZero(Extender extender) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extender = extender;
    this.timer = new Timer();
    this.timeLimit = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.restart();
    extender.gotoDefaultPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extender.isRetracted()||!extender.maxLimitSwitch()||timer.get() > timeLimit;
  }
}
