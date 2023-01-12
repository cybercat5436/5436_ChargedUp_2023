package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RoboRioPortConfig;

public class SensorTester extends SubsystemBase{
    
    AnalogInput frontLeftTurningEncoder;
    AnalogInput frontRighTurningEncoder;
    AnalogInput backLeftTurningEncoder;
    AnalogInput backRightTurningEncoder;

    // navX MXP using SPI
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SensorTester(){
        frontLeftTurningEncoder = new AnalogInput(RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_LEFT);
        frontRighTurningEncoder = new AnalogInput(RoboRioPortConfig.ABSOLUTE_ENCODER_FRONT_RIGHT);
        backLeftTurningEncoder = new AnalogInput(RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_LEFT);
        backRightTurningEncoder = new AnalogInput(RoboRioPortConfig.ABSOLUTE_ENCODER_BACK_RIGHT);
        
        // 10:39 in video to set zero
        // resetting must be delayed a bit due to calibration during instantiation
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }
            catch (Exception e) {
                System.out.println("Unable to zero the gyro during instantiation");
            }
        });
    }

    public void zeroHeading(){
        gyro.reset();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);
        builder.addDoubleProperty("FrontLeftAnalog", () -> frontLeftTurningEncoder.getVoltage(), null);
        builder.addDoubleProperty("FrontRightAnalog", () -> frontRighTurningEncoder.getVoltage(), null);
        builder.addDoubleProperty("BackLeftAnalog", () -> backLeftTurningEncoder.getVoltage(), null);
        builder.addDoubleProperty("BackRightAnalog", () -> backRightTurningEncoder.getVoltage(), null);
        builder.addDoubleProperty("Gyro", () -> gyro.getYaw(), null);
    }

    


}
