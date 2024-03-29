// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutonIntakeCommand extends CommandBase {

  private Timer timer;
  private double timeLimit = 0.0;
  private Intake intake;

  /** Creates a new AutonIntakeCommand. */
  public AutonIntakeCommand(Intake intake, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    timeLimit = time;
    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeFeedIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > timeLimit ? true : false;
  }
}
