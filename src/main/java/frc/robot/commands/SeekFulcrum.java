// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SeekFulcrum extends CommandBase {

  private double xSpeed;
  private double ySpeed = 0;
  private double turningSpeed = 0;
  private SwerveSubsystem swerveSubsystem;
  private double pitchDegrees;
  private double previousPitchDegrees;
  private double deltaPitch;
  private Timer timer;

  /** Creates a new SeekFulcrom. */
  public SeekFulcrum(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*********** Entering Seeking Fulcrum ************");
    timer.reset();
    timer.start();
    pitchDegrees = swerveSubsystem.getPitchDegrees();
    previousPitchDegrees = pitchDegrees;
    swerveSubsystem.setSaturatedPitch(pitchDegrees);
    System.out.println("Saturated Pitch set to " + pitchDegrees);
    // if (Math.abs(pitchDegrees) > 10) {
    // } 
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitchDegrees = swerveSubsystem.getPitchDegrees();

    xSpeed = 0.10 *DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond*-Math.signum(pitchDegrees);

    deltaPitch = ((pitchDegrees - previousPitchDegrees) / 20);
    SmartDashboard.putNumber("SeekFulcrum Delta Pitch", deltaPitch);
    //System.out.println(deltaPitch);
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);

    previousPitchDegrees = pitchDegrees;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Fulcrum seeked");
    System.out.println("Time Spent In SeekFulcrum: "+timer.get()+" seconds");
    swerveSubsystem.stopModules();
    swerveSubsystem.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   boolean isTilting = Math.abs(deltaPitch) > .03;
   boolean isUnsaturated = Math.abs(pitchDegrees) < Math.abs(swerveSubsystem.getSaturatedPitch()) - 2.0;
   boolean isTimedOut = timer.get()>7.0;
   if (isUnsaturated) {
    System.out.println("Fulcrum Seeked with angle " + pitchDegrees);
   }
   if (isTimedOut) {
    System.out.println("Timed out :(");
   }

   return isUnsaturated || isTimedOut;
}
}