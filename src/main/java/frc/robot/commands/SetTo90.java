// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SetTo90 extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  /** Creates a new SetTo90. */
  private Timer timer;
  private double timeLimit;
  public SetTo90(SwerveSubsystem swerveSubsystem, double timeLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    timer = new Timer();
    this.timeLimit = timeLimit;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("********initialize Move to 90**********");
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // DataLogManager.log("execute");
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.1, 0, swerveSubsystem.getRotation2d());
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 
    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i].angle = Rotation2d.fromDegrees(90);
    }
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {     
    // DataLogManager.log("end"); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // DataLogManager.log("isFinished");
    return timer.get() > timeLimit ? true : false;
  }
}
