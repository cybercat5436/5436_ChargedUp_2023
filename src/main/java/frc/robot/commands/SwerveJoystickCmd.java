package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimeLight;

public class SwerveJoystickCmd extends CommandBase {
    private SwerveSubsystem swerveSubsystem;
    private LimeLight limeLightGrid;
    private Supplier <Double> xSpdFunction, ySpdFunction, turningSpdFunction, leftTrigger, rightTrigger;
    private Supplier<Boolean> fieldOrientedFunction;
    private Supplier<Boolean> visionAdjustmentFunction;
    private Supplier<Boolean> halfSpeedFunction;
    private Supplier<Boolean> chargePadFunction;
    private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5);
    private double kLimelightHorizontal = 0.08;
    private double kLimelightForward = 1.3;
    private double kLimelightTurning =  0.1;
    private double targetHeading = 0;
    private double balanceConstant = (.0033);
    private double feedForwardConstant = (.00034);
    private double previousRoll = 0;
    private double rollROC;
    private double rollROCConstant = -4.16;
    private double errorMultiplier;
    private double superFastModeConstant = 7.5;
    private double xSpeedFast, ySpeedFast;
    private SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(superFastModeConstant * 2.0);
    private SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(superFastModeConstant * 2.0);
    private double xSpeed, ySpeed;
    private boolean isSlewActive;




    public  SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                Supplier<Double> xSpdFunction, 
                Supplier<Double> ySpdFunction, 
                Supplier<Double> turningSpdFunction,
                Supplier<Boolean> fieldOrientedFunction,
                Supplier<Boolean> halfSpeedFunction,
                Supplier<Boolean> chargePadFunction,
                Supplier<Boolean> visionAdjustmentFunction, 
                Supplier<Double> leftTrigger,
                Supplier<Double> rightTrigger,
                LimeLight limeLight){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.chargePadFunction = chargePadFunction;
        this.halfSpeedFunction = halfSpeedFunction;
        this.addRequirements(swerveSubsystem);
        this.visionAdjustmentFunction = visionAdjustmentFunction;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.limeLightGrid = limeLight;

        // Register the sendable to LiveWindow and SmartDashboard
        SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
        SmartDashboard.putData(this);
    }

   

    @Override

    public void execute(){
        // get real time joyStick inputs
        // xSpeed = xSpdFunction.get();
        // ySpeed = ySpdFunction.get();
        //can not implement sleRateLimiter =.calc because we need x and y values within execute 
;
       // System.out.println("THIS IS XSPEED FAST:" + xSpeedFast);


        double turningSpeed = turningSpdFunction.get();
        isSlewActive = rightTrigger.get() > 0.2;

        boolean targetInView = limeLightGrid.getVisionTargetStatus();
        boolean autoVisionFunction = visionAdjustmentFunction.get();
                boolean halfSpeed = halfSpeedFunction.get();

        // Read in the robot xSpeed from controller
        if (Math.abs(xSpeed) > OIConstants.K_DEADBAND) {
            xSpeed = Math.pow(xSpeed, 2) * Math.signum(xSpeed);
           
            if (Math.abs(rightTrigger.get())> .2) {
                
                 xSpeed *= superFastModeConstant;
                 System.out.println("This is xspeed" + xSpeed);

            } else {
                xSpeed *= DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
            }
        } else {
            xSpeed = 0.0;
        }
        xSpeed = processRawDriveSignal(xSpdFunction.get());
        xSpeed = applySpeedScaleToDrive(xSpeed, slewRateLimiterX);


        xSpeedFast = slewRateLimiterX.calculate(xSpeed);
        

        // Read in robot ySpeed from controller
        if (Math.abs(ySpeed) > OIConstants.K_DEADBAND){
            ySpeed = Math.pow(ySpeed, 2) * Math.signum(ySpeed);

            if(Math.abs(rightTrigger.get()) > .2){

                ySpeed *= superFastModeConstant;
                
            } else {
                ySpeed *= DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
            }
        } else {
            ySpeed = 0.0;
        }
        
        ySpeedFast = slewRateLimiterY.calculate(ySpeed); 
        if(rightTrigger.get() > 0.2) {
            xSpeed = xSpeedFast;
            ySpeed = ySpeedFast;
        }

        // Read in robot turningSpeed from controller
        if (Math.abs(turningSpeed) > OIConstants.K_DEADBAND){
            turningSpeed *= DriveConstants.kRotateDriveMaxSpeedMetersPerSecond;
        } else {
            turningSpeed = 0.0;
        }

        // Apply speed reduction if commanded
        double superSlowMo = (1 - leftTrigger.get());
        superSlowMo = Math.max(0.2, superSlowMo);
        xSpeed *= superSlowMo;
        ySpeed *= superSlowMo;
        turningSpeed *= superSlowMo;




        // make driving smoother
        // xSpeed = slewRateLimiter.calculate(xSpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // ySpeed = slewRateLimiter.calculate(ySpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // turningSpeed = slewRateLimiter.calculate(turningSpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        if(targetInView && autoVisionFunction){
            fieldOrientedFunction = () -> false;
            xSpeed = limeLightGrid.getVisionTargetAreaError() * kLimelightForward;
            ySpeed = -limeLightGrid.getVisionTargetHorizontalError() * kLimelightHorizontal;
        }else{
            fieldOrientedFunction = () -> true;
        }
        if(autoVisionFunction){
            turningSpeed=(targetHeading - swerveSubsystem.getHeading())*kLimelightTurning;
        }

 
        //rollROC = ((swerveSubsystem.getRollDegrees() - previousRoll)/20);
        if (chargePadFunction.get()) {
            xSpeed = swerveSubsystem.autoBalance();
        //    double balanceError = 0 - swerveSubsystem.getRollDegrees();
        //     if (balanceError < 0) {
        //         errorMultiplier = -1;
        //     } else {
        //         errorMultiplier = 1;
        //     }
        //     double sqrBalanceError = (Math.pow(balanceError, 2)) * errorMultiplier;
            
        //     //rollROC (rate of change) is in Degrees/Milisecond
        //     double proportionalSpeed = (balanceConstant * balanceError) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
        //     //deriv speed -4.16 proportional speed .0033
        //     double derivSpeed = ((rollROC * rollROCConstant) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
        //     double feedForwardSpeed = ((feedForwardConstant * sqrBalanceError) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
        //     //derivSpeed = Math.min(Math.abs(proportionalSpeed + feedForwardSpeed), Math.abs(derivSpeed)) * Math.signum(derivSpeed);
        //     SmartDashboard.putNumber("proportional speed", proportionalSpeed);
        //     SmartDashboard.putNumber("deriv Speed", derivSpeed);
        //     SmartDashboard.putNumber("feed forward speed", feedForwardSpeed);
        //     xSpeed = proportionalSpeed + derivSpeed + feedForwardSpeed;
        //     previousRoll = swerveSubsystem.getRollDegrees();
    
        }

        /**  if(chargePadFunction.get()){

            double balanceError = 0 - swerveSubsystem.getRollDegrees();
            xSpeed = (balanceConstant * balanceError) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
            xSpeed += ((rollROC * rollROCConstant) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
            previousRoll = swerveSubsystem.getRollDegrees();

        } */
        
        
        // convert speeds to reference frames
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()){
            //need to define in constants.java
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            
            
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
        for(int i = 0; i< moduleStates.length; i++){

            // DataLogManager.log(String.format("module %d %f", i, moduleStates[i].speedMetersPerSecond));

        }
        

        SmartDashboard.putBoolean("fieldOrientedFlag", fieldOrientedFunction.get());
        SmartDashboard.putNumber("Left Stick Y", xSpeed);
        SmartDashboard.putNumber("Left Stick X", ySpeed);
        SmartDashboard.putNumber("Drive Angle", Math.atan2(ySpeed,xSpeed));
        SmartDashboard.putNumber("vxSpeed", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vySpeed", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("kLimelightHorizontal", kLimelightHorizontal);
        SmartDashboard.putNumber("kLimelightForward", kLimelightForward);
        SmartDashboard.putBoolean("targetInView", limeLightGrid.getVisionTargetStatus());
        SmartDashboard.putBoolean("autoVisionFunction", visionAdjustmentFunction.get());
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putBoolean(" half speed", halfSpeed);
    


    }

    /**
     * Process the controller input into the drive command by applying deadband and squaring
     * @param rawDriveSignal
     * @return
     */
    private double processRawDriveSignal(double rawDriveSignal){
        // Read in the robot xSpeed from controller
        boolean isOutsideDeadband = Math.abs(rawDriveSignal) > OIConstants.K_DEADBAND;
        double speed = isOutsideDeadband ? rawDriveSignal : 0.0;
        return Math.pow(speed,2) * Math.signum(speed);
    }

    private double applySpeedScaleToDrive(double processedDriveSignal, SlewRateLimiter slewRateLimiter){
        double requestedSpeed = processedDriveSignal *  DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
        double slewedSpeed = slewRateLimiter.calculate(requestedSpeed);
        return isSlewActive ? slewedSpeed : requestedSpeed;
    }

    @Override
    public void end(boolean interupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method super.initSendable(builder);
        builder.addDoubleProperty("kLimeLightHorizontal", () -> kLimelightHorizontal, (value) -> kLimelightHorizontal = value);
        builder.addDoubleProperty("kLimeLightForward", () -> kLimelightForward, (value) -> kLimelightForward = value);
        builder.addBooleanProperty("targetInView", () -> limeLightGrid.getVisionTargetStatus(), null);
        builder.addBooleanProperty("autoVisionFunction", () -> visionAdjustmentFunction.get(), null);
        builder.addDoubleProperty("kLimeLightTurning", () -> kLimelightTurning, (value) -> kLimelightTurning = value);
        builder.addDoubleProperty("targetHeading", () -> targetHeading, (value) -> targetHeading = value);
        builder.addDoubleProperty("x speed", () -> xSpeed, (value) -> xSpeed = value);
        builder.addDoubleProperty("Y speed", () -> ySpeed, (value) -> ySpeed = value);
        builder.addDoubleProperty("rightTrigger.get()",() -> rightTrigger.get(), null);

    }

}
