package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
    private SwerveSubsystem swerveSubsystem;
    private Supplier <Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private Supplier<Boolean> fieldOrientedFunction;
    private Supplier<Boolean> halfSpeedFunction;
    private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5);
    private XboxController xboxController;


    public  SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
                Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
                Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> halfSpeedFunction){
                    this.swerveSubsystem = swerveSubsystem;
                    this.xSpdFunction = xSpdFunction;
                    this.ySpdFunction = ySpdFunction;
                    this.turningSpdFunction = turningSpdFunction;
                    this.fieldOrientedFunction = fieldOrientedFunction;
                    this.halfSpeedFunction = halfSpeedFunction;
                    this.addRequirements(swerveSubsystem);

                }
                

    @Override

    public void execute(){
        // get real time joyStick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
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

        if(halfSpeed){
            xSpeed /= 2;
            ySpeed /= 2;
            turningSpeed /= 2;
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

            DataLogManager.log(String.format("module %d %f", i, moduleStates[i].speedMetersPerSecond));

        }
        

        SmartDashboard.putBoolean("fieldOrientedFlag", fieldOrientedFunction.get());
        SmartDashboard.putNumber("Left Stick Y", xSpeed);
        SmartDashboard.putNumber("Left Stick X", ySpeed);
        SmartDashboard.putNumber("Drive Angle", Math.atan2(ySpeed,xSpeed));
        SmartDashboard.putNumber("vxSpeed", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vySpeed", chassisSpeeds.vyMetersPerSecond);


    }

    @Override
    public void end(boolean interupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
