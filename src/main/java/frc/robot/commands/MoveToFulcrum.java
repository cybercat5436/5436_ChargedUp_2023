// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToFulcrum extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  private double centerOfMassHeight = 0.6;
  private double saturatedPitch;
  private double distanceConstant = 0.05;
  private double xSpeed;
  private double kPDistance = 0.25;
  private Timer timer;
  private double timeLimit = 2;
  private double distanceError;
  

  /** Creates a new MoveToFulcrum. */
  public MoveToFulcrum(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    saturatedPitch = swerveSubsystem.getSaturatedPitch();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaPitch = swerveSubsystem.getPitchDegrees() - saturatedPitch;
    double targetDistance = centerOfMassHeight*Math.tan(Math.toRadians(deltaPitch))-distanceConstant;
    ArrayList<SwerveModule> swerveModules = swerveSubsystem.getSwerveModules();
    double dSum = 0;
    for(SwerveModule x: swerveModules){
      dSum+=x.getDrivePosition();
    }
    distanceError = targetDistance - dSum/4.0;
    xSpeed = distanceError*kPDistance;

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(xSpeed, 0, 0);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    System.out.println("Exiting MoveToFulcrum");
    System.out.println("Time spent in MoveToFulcrum: "+timer.get()+" seconds.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get()<timeLimit) return false;
    if(distanceError<0.01){
      System.out.println("Successfully moved to fulcrum.");
      return true;
    }else if(Math.abs(swerveSubsystem.getPitchDegrees())>8){
      System.out.println("Error is saturated.");
      return true;
    }else{
      return false;
    }
  }
}
