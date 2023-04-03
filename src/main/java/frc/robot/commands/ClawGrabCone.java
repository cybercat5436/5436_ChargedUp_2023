// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawGrabCone extends CommandBase {
  /** Creates a new ClawGrabCone. */
  private Claw claw;
  private Timer timer;
  private double timeLimit = 1.2;
  
  public ClawGrabCone(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
    timer = new Timer();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // DataLogManager.log("ClawGrab Initialize");
    claw.grabCone();
    timer.reset();
    timer.start();
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
    return claw.isConeGrabbed()||timer.get()>timeLimit;
  }
}
