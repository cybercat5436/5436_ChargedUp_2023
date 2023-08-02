// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;

public class AutoAlign extends CommandBase {
  private LimeLight limeLight;
  private boolean inSight = false;
  private double tx;
  private double ty;
  private double tz;
  private double targetTx = -1.0;
  private double targetTz = -1.8;
  private double txError;
  private double tzError;
  private double[] botpose;
  
  /** Creates a new AutoAlign. */
  public AutoAlign(LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!limeLight.getVisionTargetStatus()){
      System.out.println("AprilTag is not in sight.");
    }else{
      inSight = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limeLight.getVisionTargetStatus()){
      // for(double x:limeLight.getBotPose()){
      //   System.out.print(x+" ");
      // }
      // System.out.println();
      botpose = limeLight.getBotPose();
      tx = botpose[0];
      ty = botpose[1];
      tz = botpose[2];
      
      txError = tx - targetTx;
      tzError = tz - targetTz;
      System.out.println("txError: "+txError);
      System.out.println("tzError: "+tzError);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
