// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToFulcrum extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  private double centerOfMassHeight = 0.4;
  private double saturatedPitch;
  private double distanceConstant = 0.080;
  private double xSpeed;
  private double kPDistance = 1.0;
  private Timer timer;
  private double minTimeInState = 2.0;
  private double maxTimeInState = 5.0;
  private double distanceError;
  private double allowableError = 0.01;
  private int targetAchievedCount = 0;
  

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
    double targetDistance = centerOfMassHeight*Math.tan(Math.toRadians(deltaPitch));
    boolean isDistanceConstantGreaterThanTarget = distanceConstant > Math.abs(targetDistance);
    double distanceConstantSign = -Math.signum(saturatedPitch);
    targetDistance = isDistanceConstantGreaterThanTarget ? 0.0 : -(targetDistance - (distanceConstant * distanceConstantSign));

    // quantify how far the chassis has moved
    ArrayList<SwerveModule> swerveModules = swerveSubsystem.getSwerveModules();
    double dSum = 0;
    for(SwerveModule x: swerveModules){
      dSum+=x.getDrivePosition();
    }
    // quantify distance error
    distanceError = targetDistance - dSum/4.0;
    // System.out.println("DistError:   " + distanceError);
    xSpeed = distanceError * kPDistance * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
    // SmartDashboard.putNumber("Target Distance: " + targetDistance + "  -- MoveToFulcrum xSpeed", xSpeed);
    // System.out.println("deltaPitch: " + deltaPitch);
    // System.out.println("Target Distance:  " + targetDistance + "  ****  XSpeed =  " + xSpeed);
    
    // Count successive cycles where target achieved.
    if(Math.abs(distanceError) < allowableError){
      targetAchievedCount++;
    } else{
      targetAchievedCount = 0;
    }

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
    System.out.println("Time spent in MoveToFulcrum: " + timer.get() + " seconds.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // don't exit at they very beginning, allow the charge pad to tip
    if(timer.get() < minTimeInState) return false;
    
    // check if target position is sustained
    if(targetAchievedCount >= 5){
      System.out.println("Successfully moved to fulcrum.");
      return true;
    }

    // exit if timed out
    if(timer.get() > maxTimeInState){
      System.out.println("Moved to fulcrum timed out.");
      return true;
    }
    
    // check if pitch error is too large
    if(isPitchErrorLarge()){
      System.out.println("Moved to fulcrum exited because pitch error to large.");
      return true;
    }

    return false;
  }

  private boolean isPitchErrorLarge(){
        // check if pitch error is too large
        double errorThreshold = 8.0;
        double currentPitch = swerveSubsystem.getPitchDegrees();
        double pitchError = swerveSubsystem.getTargetPitch() - currentPitch;
        
        if(Math.abs(pitchError) > errorThreshold){
          System.out.println("Error is saturated, pitch is " + currentPitch + 
          " with target of: " + swerveSubsystem.getTargetPitch() +
          " for error of: " + pitchError);
          return true;
        }else{
          return false;
        }

        
  }
}
