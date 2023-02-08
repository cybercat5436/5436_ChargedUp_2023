// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonomousDriveCommand extends CommandBase {
  /** Creates a new AutonomousDriveCommand. */
private final SwerveSubsystem swerveSubsystem;
private double xSpeed;
private double ySpeed;
private double turningSpeed;
private Timer timer;
private double timeLimit;

  public AutonomousDriveCommand(SwerveSubsystem swerveSubsystem, double timeLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = 0;
    this.ySpeed = 0;
    this.turningSpeed = 0;
    timer = new Timer();
    this.timeLimit = timeLimit;
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

    //SmartDashboard.putNumber(timer.get());
    this.xSpeed = swerveSubsystem.autoBalance();

  //   if (Math.abs(xSpeed) > OIConstants.K_DEADBAND) {
  //     xSpeed *= DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
  // } else {
  //     xSpeed = 0.0;
  // }
  // //ySpeed = Math.abs(ySpeed) > OIConstants.K_DEADBAND ? ySpeed : 0.0 *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
  // if (Math.abs(ySpeed) > OIConstants.K_DEADBAND){
  //     ySpeed *= DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
  // } else {
  //     ySpeed = 0.0;
  // }

  // //turningSpeed = Math.abs(turningSpeed) > OIConstants.K_DEADBAND ? turningSpeed : 0.0;

  // if (Math.abs(turningSpeed) > OIConstants.K_DEADBAND){
  //     turningSpeed *= DriveConstants.kRotateDriveMaxSpeedMetersPerSecond;
  // } else {
  //     turningSpeed = 0.0;
  // }
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //WRITE EXIT CONDITION BASED ON HOW MANY CYCLES IT'S BALANCED 
    //TIME BASED EXIT CONDITION
    return timer.get() > timeLimit ? true : false;
  }
}
