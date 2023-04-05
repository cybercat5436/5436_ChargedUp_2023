// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;

public class ZeroExtender extends CommandBase {
  private Extender extender;
  private Timer timer = new Timer();


  /** Creates a new ZeroExtender. */
  public ZeroExtender(Extender extender) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extender = extender;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    extender.retract(0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extender.stopExtend();
    extender.resetExtenderEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>0.3;
  }
}
