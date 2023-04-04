package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmGoToHigh2;
import frc.robot.commands.ArmGoToHighMotionMagic;
import frc.robot.commands.ClawGrabCone;
import frc.robot.commands.ClawReset;
import frc.robot.commands.ExtendHighGoal;
import frc.robot.commands.ExtenderRetractToZero;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.ZeroExtender;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Utils {
    public Utils(){

    }
    public SwerveControllerCommand getSwerveControllerCommand(Trajectory trajectory, SwerveSubsystem swerveSubsystem){
        ProfiledPIDController thetaController = swerveSubsystem.getThetaController();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            swerveSubsystem.getxController(),
            swerveSubsystem.getyController(),
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem); 
    }

    public Trajectory getTrajectory(String path){
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
            System.out.println("Unable to open trajectory" + path);
          }

        return trajectory;
    }

    public SequentialCommandGroup autonDriveCommand(String string, SwerveSubsystem swerveSubsystem){
      Trajectory trajectory = getTrajectory(string);
      SwerveControllerCommand swerveControllerCommand = getSwerveControllerCommand(trajectory, swerveSubsystem);

      return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
        new ManualEncoderCalibration(swerveSubsystem),
        swerveControllerCommand, 
        new InstantCommand(() -> swerveSubsystem.stopModules()));
      

    }

  public SequentialCommandGroup scoreHighGoal(Extender extender, Claw claw, Arm arm){

      return new SequentialCommandGroup(
        new InstantCommand(()->arm.resetArmEncoder()),
        new ZeroExtender(extender),
        new ClawGrabCone(claw),
        new ArmGoToHighMotionMagic(arm),
        new ExtendHighGoal(extender, 2.0));
    }

    public SequentialCommandGroup retractArm(Extender extender, Claw claw, Arm arm){

      return new SequentialCommandGroup(
        new ArmGoToHigh2(arm),
        new ClawReset(claw),
        new ExtenderRetractToZero(extender),
        new InstantCommand(()->arm.armMoveToZeroPosition()));
      
    }

  
    
}
