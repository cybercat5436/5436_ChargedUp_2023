// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmGoToHighMotionMagic extends CommandBase {
  private Arm arm;
 
  /** Creates a new ArmGoToHigh. */
  public ArmGoToHighMotionMagic(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("Arm Go To High Motion Magic Initialised");
    arm.armHighGoalMotionMagic();
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
    // return arm.isAtHighGoal();
    return arm.hasExitedChassis();
  }
}
