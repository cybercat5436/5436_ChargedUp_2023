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
import edu.wpi.first.wpilibj.Timer;
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
    private double kLimelightHorizontal = 0.08;
    private double kLimelightForward = 1.3;
    private double kLimelightTurning =  0.1;
    private double targetHeading = 0;
    // private double superFastModeConstant = 7.5;
    private SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 2.0);
    private SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 2.0);
    private SlewRateLimiter slewRateLimiterTheta = new SlewRateLimiter(DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 2.0);
    private double xSpeed, ySpeed, turningSpeed;
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
        this.visionAdjustmentFunction = visionAdjustmentFunction;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.limeLightGrid = limeLight;

        this.addRequirements(swerveSubsystem);


        // Register the sendable to LiveWindow and SmartDashboard
        SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
        SmartDashboard.putData(this);
    }

   

    @Override

    public void execute(){

        isSlewActive = rightTrigger.get() > 0.2;

        boolean targetInView = limeLightGrid.getVisionTargetStatus();
        boolean autoVisionFunction = visionAdjustmentFunction.get();

        // Read in the robot xSpeed from controller
        xSpeed = processRawDriveSignal(xSpdFunction.get());
        xSpeed = applySpeedScaleToDrive(xSpeed);
        xSpeed = applySlewRateLimiter(xSpeed, slewRateLimiterX);
        
        // Read in the robot ySpeed from controller
        ySpeed = processRawDriveSignal(ySpdFunction.get());
        ySpeed = applySpeedScaleToDrive(ySpeed);
        ySpeed = applySlewRateLimiter(ySpeed, slewRateLimiterY);
    

        // Read in robot turningSpeed from controller
        turningSpeed = processRawDriveSignal(turningSpdFunction.get());
        turningSpeed = applySpeedScaleToDrive(turningSpeed);
        turningSpeed = applySlewRateLimiter(turningSpeed, slewRateLimiterTheta);

        // Apply speed reduction if commanded
        double superSlowMo = (1 - leftTrigger.get());
        superSlowMo = Math.max(0.2, superSlowMo);
        xSpeed *= superSlowMo;
        ySpeed *= superSlowMo;
        turningSpeed *= superSlowMo;

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
        }
        
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

            SmartDashboard.putNumber(String.format("module %d", i), moduleStates[i].speedMetersPerSecond);

        }
        
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

    private double applySpeedScaleToDrive(double processedDriveSignal){
        // If right trigger is pulled, used max speed, otherwise use translate speed
        double speedMultiplier = isSlewActive ? DriveConstants.kPhysicalMaxSpeedMetersPerSecond : DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
        return processedDriveSignal *  speedMultiplier;
    }

    private double applySlewRateLimiter(double scaledDriveSpeed, SlewRateLimiter slewRateLimiter){
        // If right trigger is pulled, return slewRate figure, otherwise use scaled speed
        double slewedSpeed = slewRateLimiter.calculate(scaledDriveSpeed);
        return isSlewActive ? slewedSpeed : scaledDriveSpeed;
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
        // builder.addDoubleProperty("kLimeLightHorizontal", () -> kLimelightHorizontal, (value) -> kLimelightHorizontal = value);
        // builder.addDoubleProperty("kLimeLightForward", () -> kLimelightForward, (value) -> kLimelightForward = value);
        // builder.addBooleanProperty("targetInView", () -> limeLightGrid.getVisionTargetStatus(), null);
        // builder.addBooleanProperty("autoVisionFunction", () -> visionAdjustmentFunction.get(), null);
        // builder.addDoubleProperty("kLimeLightTurning", () -> kLimelightTurning, (value) -> kLimelightTurning = value);
        // builder.addDoubleProperty("targetHeading", () -> targetHeading, (value) -> targetHeading = value);
        builder.addDoubleProperty("x speed", () -> xSpeed, (value) -> xSpeed = value);
        builder.addDoubleProperty("Y speed", () -> ySpeed, (value) -> ySpeed = value);
        // builder.addDoubleProperty("FPGA Clock", () -> Timer.getFPGATimestamp(), null);
        // builder.addDoubleProperty("rightTrigger.get()",() -> rightTrigger.get(), null);

    }

}
