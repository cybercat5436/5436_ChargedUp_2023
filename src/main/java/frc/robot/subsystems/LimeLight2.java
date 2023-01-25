package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight2 extends SubsystemBase {
  private NetworkTable tableLimelight;
  private NetworkTableEntry txLocal; //horizontal error
  private NetworkTableEntry tyLocal; //vertical error
  private NetworkTableEntry taLocal; //area error
  private NetworkTableEntry tvLocal; //valid target found
  private NetworkTableEntry tsLocal; //skew error

  private double horizontalError = 0.0;
  private double verticalError = 0.0;
  private double area = 0.0;
  private double xOffset;
  private double aim = -0.1;
  private double distance = -0.1;
  private double min_aim = -0.1;
  private double visionSpeed;
  private boolean targetInView = false;

  public LimeLight2() {
    tableLimelight = NetworkTableInstance.getDefault().getTable("limelight");
    txLocal = tableLimelight.getEntry("tx"); // communicates horizontal degree offset from target
    tyLocal = tableLimelight.getEntry("ty"); // communicates verticle degree offset from target
    taLocal = tableLimelight.getEntry("ta"); // communicates percentage of image the target takes up
    tvLocal = tableLimelight.getEntry("tv"); // communicates whether a valid target is acquired, 0 or 1
    tsLocal = tableLimelight.getEntry("ts"); // communicates skew offset from target
  
  
}

@Override
  public void periodic() {
    horizontalError = getVisionTargetHorizontalError();
    verticalError = getVisionTargetVerticalError();
    area = getVisionTargetAreaError();
    targetInView = getVisionTargetStatus();
    SmartDashboard.putBoolean("Valid Target Found", targetInView);
    SmartDashboard.putNumber("tx", getVisionTargetHorizontalError());
    SmartDashboard.putNumber("ty", getVisionTargetVerticalError());
  }

 public boolean alignToTarget(boolean targetFound, double xError, double yError, String zone){
   
    double yOffset = 0;
    boolean targetAligned = false;
    boolean headingAligned = false;
    boolean distanceAligned = false;
    double vertical_error = 0;
    double steering_adjust = 0;
    double distance_adjust = 0;
    
    
    double xSpeedAdjust = 0;
    double ySpeedAdjust = 0;
    double turningSpeedAdjust = 0;
    
    //Meadow calibrated the limelight cross-hair at ideal distance for first zone to target we shoot from.
    //this meant ty = 0 at learned position at Blue zone.  Team did zones Target->Green->Blue->Yellow->Red
    //Meadow then recorded the different ty "offset" values at each of the other shooter positions.
    //Note - in Green zone we are too close to see target so not used

    vertical_error = - (yError - yOffset);  //error goes to zero as we approach our offset positions and negative accounts for drivetrain
    

    //Calculating the speed at which to rotate based on tx
    /*if (targetFound == true){ //Valid Target Found
      //Clamp speeds to never go below minimum per constants file
      steering_adjust = Math.signum(xError)* (Math.max(VisionConstants.kpAim*Math.abs(xError), VisionConstants.kMinRotateSpeed));
      distance_adjust = Math.signum(verticalError) * (Math.max(VisionConstants.kpVertical*Math.abs(vertical_error), VisionConstants.kMinLinearSpeed));
    } else {  //Target not within range
      steering_adjust = 0;
      distance_adjust = 0;
    } */

    /** 
    //Drive the Robot with the adjusted speeds simultaneously
    double leftSideSpeed = distance_adjust + steering_adjust;
    double rightSideSpeed = distance_adjust - steering_adjust;
    drive.tankDrivePWM(leftSideSpeed, rightSideSpeed);
    **/

    //Check to see if we are in range for heading and turn first
      /*if (Math.abs(xError) <= VisionConstants.kAngleThreshold){// Target is within angle threshold
        headingAligned = true;
      } else {
        headingAligned = false;
        drive.tankDrivePWM(steering_adjust, -steering_adjust);
      }

      //Check to seeif we are in range for distance once we are aligned to heading (based on vertical)
      if (headingAligned && (vertical_error == VisionConstants.kVerticalThreshold)){// Target is within vertical threshold
        distanceAligned = true;
      } else {
        distanceAligned = false;
        drive.tankDrivePWM(distance_adjust, distance_adjust);
      }*/
    
    //Set Exit Flag only once both are aligned
    if (headingAligned && distanceAligned){
      targetAligned = true;
    }else{
      targetAligned = false;
    }

    return targetAligned;

  }  //End of AlignToTargetMethod


  public boolean getVisionTargetStatus(){
    boolean returnValue = false;

    if (tvLocal.getDouble(0) == 1){
      returnValue = true;
    }

    return returnValue;
  }

  public double getVisionTargetHorizontalError(){
    return txLocal.getDouble(0);
  }

  public double getVisionTargetAreaError(){
    return 0.9-taLocal.getDouble(0);
  }

public double getVisionTargetVerticalError(){
  return tyLocal.getDouble(0);
}

public double getVisionTargetSkew(){
  return tsLocal.getDouble(0);
}



}
