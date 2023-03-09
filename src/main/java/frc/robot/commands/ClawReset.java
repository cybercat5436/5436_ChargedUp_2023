// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawReset extends CommandBase {
  /** Creates a new ClawReset. */
  private Claw claw;
  private Timer timer;
  private double timeLimit;
  public ClawReset(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    this.timer = new Timer();
    this.timeLimit = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("Claw Reset Initialise");
    timer.reset();
    timer.restart();
    claw.gotoDefaultPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.getClawPosition()<=60 || timer.get() > timeLimit;
  }
}
