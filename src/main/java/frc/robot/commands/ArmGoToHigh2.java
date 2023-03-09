// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmGoToHigh2 extends CommandBase {
  private Arm arm;
  private Timer timer;
  private double timeLimit;
  /** Creates a new ArmGoToHigh. */
  public ArmGoToHigh2(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.timer = new Timer();
    this.timeLimit = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //arm.slowMaxSpeed();
    DataLogManager.log("Arm Go to High2 Initialised");
    timer.reset();
    timer.restart();
    arm.armHighGoal2();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //arm.restoreMaxSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return arm.isAtHighGoal();
    return arm.isAtHighGoal2() || timer.get() > timeLimit;
  }
}
