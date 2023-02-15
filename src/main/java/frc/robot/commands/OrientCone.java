// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight2;
import frc.robot.subsystems.Orienter;

public class OrientCone extends CommandBase {
  /** Creates a new OrientCone. */
  Orienter orienter;
  LimeLight2 limeLight2;
  public OrientCone(Orienter orienter, LimeLight2 limeLight2) {
    this.orienter = orienter;
    this.limeLight2 = limeLight2;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    orienter.microwaveSpin();
    System.out.println("executed");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orienter.stopMicrowave();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limeLight2.isOriented();
  
  }
}
