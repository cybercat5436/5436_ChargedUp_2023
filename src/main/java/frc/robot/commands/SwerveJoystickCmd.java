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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimeLight2;

public class SwerveJoystickCmd extends CommandBase {
    private SwerveSubsystem swerveSubsystem;
    private LimeLight2 visionSubsystem;
    private Supplier <Double> xSpdFunction, ySpdFunction, turningSpdFunction;
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


    public  SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                Supplier<Double> xSpdFunction, 
                Supplier<Double> ySpdFunction, 
                Supplier<Double> turningSpdFunction,
                Supplier<Boolean> fieldOrientedFunction,
                Supplier<Boolean> halfSpeedFunction,
                Supplier<Boolean> chargePadFunction,
                Supplier<Boolean> visionAdjustmentFunction, 
                LimeLight2 limeLight2){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.chargePadFunction = chargePadFunction;
        this.halfSpeedFunction = halfSpeedFunction;
        this.addRequirements(swerveSubsystem);
        this.visionAdjustmentFunction = visionAdjustmentFunction;
        visionSubsystem = limeLight2;

        // Register the sendable to LiveWindow and SmartDashboard
        SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
        SmartDashboard.putData(this);
    }
                

    @Override

    public void execute(){
        // get real time joyStick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        boolean targetInView = visionSubsystem.getVisionTargetStatus();
        boolean autoVisionFunction = visionAdjustmentFunction.get();
                boolean halfSpeed = halfSpeedFunction.get();

        //apply dead band 
        //xSpeed = Math.abs(xSpeed) > OIConstants.K_DEADBAND ? xSpeed : 0.0 *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        if (Math.abs(xSpeed) > OIConstants.K_DEADBAND) {
            xSpeed *= DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
        } else {
            xSpeed = 0.0;
        }
        //ySpeed = Math.abs(ySpeed) > OIConstants.K_DEADBAND ? ySpeed : 0.0 *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        if (Math.abs(ySpeed) > OIConstants.K_DEADBAND){
            ySpeed *= DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
        } else {
            ySpeed = 0.0;
        }

        //turningSpeed = Math.abs(turningSpeed) > OIConstants.K_DEADBAND ? turningSpeed : 0.0;

        if (Math.abs(turningSpeed) > OIConstants.K_DEADBAND){
            turningSpeed *= DriveConstants.kRotateDriveMaxSpeedMetersPerSecond;
        } else {
            turningSpeed = 0.0;
        }

        // make driving smoother
        // xSpeed = slewRateLimiter.calculate(xSpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // ySpeed = slewRateLimiter.calculate(ySpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // turningSpeed = slewRateLimiter.calculate(turningSpeed) *DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        if(targetInView && autoVisionFunction){
            fieldOrientedFunction = () -> false;
            xSpeed = visionSubsystem.getVisionTargetAreaError() * kLimelightForward;
            ySpeed = -visionSubsystem.getVisionTargetHorizontalError() * kLimelightHorizontal;
        }else{
            fieldOrientedFunction = () -> true;
        }
        if(autoVisionFunction){
            turningSpeed=(targetHeading - swerveSubsystem.getHeading())*kLimelightTurning;
        }

        if(halfSpeed){
            xSpeed *= .3;
            ySpeed *= .3;
            turningSpeed *= .3;
        }



        rollROC = ((swerveSubsystem.getRollDegrees() - previousRoll)/20);
        if (chargePadFunction.get()) {
            double balanceError = 0 - swerveSubsystem.getRollDegrees();
            if (balanceError < 0) {
                errorMultiplier = -1;
            } else {
                errorMultiplier = 1;
            }
            double sqrBalanceError = (Math.pow(balanceError, 2)) * errorMultiplier;
            
            //rollROC (rate of change) is in Degrees/Milisecond
            double proportionalSpeed = (balanceConstant * balanceError) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond;
            //deriv speed -4.16 proportional speed .0033
            double derivSpeed = ((rollROC * rollROCConstant) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
            double feedForwardSpeed = ((feedForwardConstant * sqrBalanceError) * DriveConstants.kTranslateDriveMaxSpeedMetersPerSecond);
            //derivSpeed = Math.min(Math.abs(proportionalSpeed + feedForwardSpeed), Math.abs(derivSpeed)) * Math.signum(derivSpeed);
            SmartDashboard.putNumber("proportional speed", proportionalSpeed);
            SmartDashboard.putNumber("deriv Speed", derivSpeed);
            SmartDashboard.putNumber("feed forward speed", feedForwardSpeed);
            xSpeed = proportionalSpeed + derivSpeed + feedForwardSpeed;
            previousRoll = swerveSubsystem.getRollDegrees();
    
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
        SmartDashboard.putBoolean("targetInView", visionSubsystem.getVisionTargetStatus());
        SmartDashboard.putBoolean("autoVisionFunction", visionAdjustmentFunction.get());
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putBoolean(" half speed", halfSpeed);
    


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
        // TODO Auto-generated method stub
        super.initSendable(builder);
        builder.addDoubleProperty("kLimeLightHorizontal", () -> kLimelightHorizontal, (value) -> kLimelightHorizontal = value);
        builder.addDoubleProperty("kLimeLightForward", () -> kLimelightForward, (value) -> kLimelightForward = value);
        builder.addBooleanProperty("targetInView", () -> visionSubsystem.getVisionTargetStatus(), null);
        builder.addBooleanProperty("autoVisionFunction", () -> visionAdjustmentFunction.get(), null);
        builder.addDoubleProperty("kLimeLightTurning", () -> kLimelightTurning, (value) -> kLimelightTurning = value);
        builder.addDoubleProperty("targetHeading", () -> targetHeading, (value) -> targetHeading = value);
        builder.addDoubleProperty("balanceConstant", () -> balanceConstant, (value) -> balanceConstant = value);
        builder.addDoubleProperty("Roll Rate of Change", () -> rollROC, null);
        builder.addDoubleProperty("Roll Rate of Change Constant", () -> rollROCConstant, (value) -> rollROCConstant = value);
        builder.addDoubleProperty("feed forward", () -> feedForwardConstant, (value) -> feedForwardConstant = value);


    }

}
