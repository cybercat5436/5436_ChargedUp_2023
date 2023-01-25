// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(0);
    private final XboxController xboxController = new XboxController(1);

    String trajectoryJSON = "paths/Rectangle.wpilib.json";
    Trajectory trajectory3 = new Trajectory();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -xboxController.getLeftY(),
        () -> -xboxController.getLeftX(),
        () -> -xboxController.getRightX(),
        () -> !xboxController.getStartButtonPressed()));
      // Configure the button bindings
      ManualEncoderCalibration manualEncoderCalibration = new ManualEncoderCalibration(swerveSubsystem);
      SmartDashboard.putData(manualEncoderCalibration);
      configureButtonBindings();
      DataLogManager.logNetworkTables(true);
      DataLogManager.start();
      DataLogManager.log("Started the DataLogManager!!!");
    
      try {
              Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
              trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
              System.out.println("Unable to open trajectory");
      }
    }
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }
  
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // 1. Create trajectory settings
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      .setKinematics(DriveConstants.kDriveKinematics);

      // 2. Generate trajectory
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
             /** start on this position*/ new Pose2d(0, 0, new Rotation2d(0)),
            /**go through these points on the way through */  
             List.of(
                        
                        new Translation2d(1.75, 1.75),
                        new Translation2d(3.5, 0),
                        new Translation2d(1.75, -1.75)),
 
    
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), trajectoryConfig);
                

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        new Pose2d(1.75, 1.75, Rotation2d.fromDegrees(0)),
                        new Pose2d(3.5, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(1.75, -1.75, Rotation2d.fromDegrees(0)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
                         trajectoryConfig);
    
    /**             List.of(
                        new Translation2d(0,1),
                        new Pose2d(0, 1, Rotation2d.fromDegrees(180)), trajectoryConfig);
                

                     /**  List.of(
                      new Translation2d(1, 0),
                      new Translation2d(1, -1),
                      new Translation2d(0,-1),
                      new Translation2d(0,0)),
              new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
              trajectoryConfig);*/
      /**Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, -1),
        new Translation2d(1, -1),
        new Translation2d(1,-1),
        new Translation2d(1,-1)),
        new Pose2d(1, -1, Rotation2d.fromDegrees(180)),
          trajectoryConfig);**/

      // 3. Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
              AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      //Set the odometry to initial pose of the trajectory
      swerveSubsystem.resetOdometry(trajectory3.getInitialPose());
      //Reset all the drive motors of the swervemodules to 0
      swerveSubsystem.resetEncoders();
      // 4. Construct command to follow trajectory
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
              trajectory3,
              swerveSubsystem::getPose,
              DriveConstants.kDriveKinematics,
              xController,
              yController,
              thetaController,
              swerveSubsystem::setModuleStates,
              swerveSubsystem);
      
     
      
      /**SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                trajectory2,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);**/

      // 5. Add some init and wrap-up, and return everything
      return new SequentialCommandGroup(
              //new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
              new InstantCommand(() -> swerveSubsystem.zeroTurningEncoders()),
              swerveControllerCommand, 
              new InstantCommand(() -> swerveSubsystem.stopModules()));
              // new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory2.getInitialPose())), swerveControllerCommand2,new InstantCommand(() -> swerveSubsystem.stopModules()));
              
  }
  }
  