// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;

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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonArmDownCommand;
import frc.robot.commands.AutonArmUpCommand;
import frc.robot.commands.AutonGrabCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.AutonReleaseCommand;
import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.ArmGoToHighMotionMagic;
import frc.robot.commands.ArmGoToHigh2;
import frc.robot.commands.ArmGoToMid;
import frc.robot.commands.ClawGrabCone;
import frc.robot.commands.ClawReset;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendHighGoal;
import frc.robot.commands.ExtenderRetractToZero;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.SetTo90;
import frc.robot.commands.OrientCone;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroExtender;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Orienter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public boolean halfSpeed = false;
    private final LimeLight limeLightGrid = new LimeLight("limelight");
    private final LimeLight limeLightOrient = new LimeLight("limelight-orient");
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Orienter orienter = new Orienter(limeLightOrient);
    private final Claw claw = new Claw();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final Extender extender = new Extender();
    private final CommandXboxController primaryController = new CommandXboxController(1);
    private final CommandXboxController secondaryController = new CommandXboxController(0);

    String trajectoryJSON = "paths/ForwardPathRight.wpilib.json";    
    Trajectory trajectory = new Trajectory();

    String trajectoryJSON2 = "paths/ForwardPathRight1.wpilib.json";
    Trajectory trajectory2 = new Trajectory();

    String trajectoryLeftPathJSON = "paths/ForwardPathLeft.wpilib.json";
    Trajectory trajectoryLeft = new Trajectory();


    String chargePad1JSON = "paths/ChargePad1.wpilib.json";
    Trajectory chargePad1Trajectory = new Trajectory();

    String chargePad2JSON = "paths/ChargePad2.wpilib.json";
    Trajectory chargePad2Trajectory = new Trajectory();

    private final ZeroExtender zeroExtender = new ZeroExtender(extender);

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    private SequentialCommandGroup scoreHighGoal = new SequentialCommandGroup(
      new ClawGrabCone(claw),
      new ArmGoToHighMotionMagic(arm),
      new InstantCommand(()->extender.extendHighGoal())
    );

    private SequentialCommandGroup scoreHighGoalAuton = new SequentialCommandGroup(
      new InstantCommand(()->arm.resetArmEncoder()),
      new ZeroExtender(extender),
      new ClawGrabCone(claw),
      new ArmGoToHighMotionMagic(arm),
      new ExtendHighGoal(extender, 2.0)
    );
    
    private SequentialCommandGroup scoreHighGoalDeliverAuton = new SequentialCommandGroup(
      new InstantCommand(()->arm.resetArmEncoder()),
      new ZeroExtender(extender),
      new ClawGrabCone(claw),
      new ArmGoToHighMotionMagic(arm),
      new ExtendHighGoal(extender, 2.0)
    );

    private SequentialCommandGroup retractArm = new SequentialCommandGroup(
      new ArmGoToHigh2(arm),
      new WaitCommand(1),
      new ClawReset(claw),
      new ExtenderRetractToZero(extender),
      new InstantCommand(()->arm.armMoveToZeroPosition()) 
    );

    private SequentialCommandGroup retractArmAuton = new SequentialCommandGroup(
      new ArmGoToHigh2(arm),
      new WaitCommand(0.5),
      new ClawReset(claw),
      new ExtenderRetractToZero(extender),
      new InstantCommand(()->arm.armMoveToZeroPosition()) 
    );

    private SequentialCommandGroup retractArmDeliverAuton = new SequentialCommandGroup(
      new ArmGoToHigh2(arm),
      new WaitCommand(0.5),
      new ClawReset(claw),
      new ExtenderRetractToZero(extender),
      new InstantCommand(()->arm.armMoveToZeroPosition()) 
    );



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -primaryController.getLeftY(),
        () -> -primaryController.getLeftX(),
        () -> -primaryController.getRightX(),
        () -> !primaryController.start().getAsBoolean(),
        // () -> primaryController.leftBumper().getAsBoolean(),
        () -> primaryController.rightTrigger().getAsBoolean(),
        () -> primaryController.y().getAsBoolean(),
        //() -> primaryController.rightBumper().getAsBoolean(),
        () -> primaryController.x().getAsBoolean(),
        () -> primaryController.getLeftTriggerAxis(),
        limeLightGrid));

      // Configure the button bindings
      ManualEncoderCalibration manualEncoderCalibration = new ManualEncoderCalibration(swerveSubsystem);        
      primaryController.b()

          .onTrue(new OrientCone(orienter, limeLightGrid));

      SmartDashboard.putData(manualEncoderCalibration);
      configureButtonBindings();

      SmartDashboard.putData(new InstantCommand(() -> swerveSubsystem.zeroIntegrator()));

      SmartDashboard.putData(zeroExtender);
      SmartDashboard.putData(new ArmGoToHighMotionMagic(arm));

      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        System.out.println("Unable to open trajectory" + ex);
      }

      
      try {
        Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
        trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
      } catch (IOException ex) {
        System.out.println("Unable to open ForwardPathRight1 trajectory " + ex);
      }


      try {
        Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryLeftPathJSON);
        trajectoryLeft = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
      } catch (IOException ex) {
        System.out.println("Unable to open ForwardLeftPath trajectory " + ex);
      }

      try {
        Path chargePadTrajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(chargePad1JSON);
        chargePad1Trajectory = TrajectoryUtil.fromPathweaverJson(chargePadTrajectoryPath1);
      } catch (IOException ex) {
        System.out.print("unable to open charge pad 1" + ex);
      }

      try {
        Path chargePadTrajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(chargePad2JSON);
        chargePad2Trajectory = TrajectoryUtil.fromPathweaverJson(chargePadTrajectoryPath2);
      } catch (IOException ex) {
        System.out.print("unable to open charge pad 1" + ex);
      }

      //create auton commands

      ProfiledPIDController thetaController = swerveSubsystem.getThetaController();
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      //Set the odometry to initial pose of the trajectory
      swerveSubsystem.resetOdometry(trajectory.getInitialPose());
      //Reset all the drive motors of the swervemodules to 0
      swerveSubsystem.resetEncoders();

      SwerveControllerCommand autoBalanceTrajectoryCommand = new SwerveControllerCommand(
        chargePad1Trajectory.concatenate(chargePad2Trajectory),
          swerveSubsystem::getPose,
          DriveConstants.kDriveKinematics,
          swerveSubsystem.getxController(),
          swerveSubsystem.getyController(),
          thetaController,
          swerveSubsystem::setModuleStates,
          swerveSubsystem); 
      
      //Trajectory to Drive to Pad
      SequentialCommandGroup autonDriveToPad = new SequentialCommandGroup(
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(chargePad1Trajectory.getInitialPose())), 
                    new ManualEncoderCalibration(swerveSubsystem),
                    autoBalanceTrajectoryCommand,  
                    new InstantCommand(() -> swerveSubsystem.stopModules()));
   

        SwerveControllerCommand rightRoutineCommand = new SwerveControllerCommand(
          trajectory.concatenate(trajectory2),
          swerveSubsystem::getPose,
          DriveConstants.kDriveKinematics,
          swerveSubsystem.getxController(),
          swerveSubsystem.getyController(),
          thetaController,
          swerveSubsystem::setModuleStates,
          swerveSubsystem); 
                
        SequentialCommandGroup autonForwardPath = new SequentialCommandGroup(
                  //new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
                  new ManualEncoderCalibration(swerveSubsystem),
                  rightRoutineCommand,  
                  new InstantCommand(() -> swerveSubsystem.stopModules()));   
                  
                  
        SwerveControllerCommand leftRoutineCommand = new SwerveControllerCommand(
                    trajectoryLeft,
                    swerveSubsystem::getPose,
                    DriveConstants.kDriveKinematics,
                    swerveSubsystem.getxController(),
                    swerveSubsystem.getyController(),
                    thetaController,
                    swerveSubsystem::setModuleStates,
                    swerveSubsystem);
        
        SwerveControllerCommand leftDriveAndDeliverCommand = new SwerveControllerCommand(
                      trajectoryLeft,
                      swerveSubsystem::getPose,
                      DriveConstants.kDriveKinematics,
                      swerveSubsystem.getxController(),
                      swerveSubsystem.getyController(),
                      thetaController,
                      swerveSubsystem::setModuleStates,
                      swerveSubsystem); 
                          
        SequentialCommandGroup autonForwardLeftPath = new SequentialCommandGroup(
                            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryLeft.getInitialPose())), 
                            new ManualEncoderCalibration(swerveSubsystem),
                            leftRoutineCommand,  
                            new InstantCommand(() -> swerveSubsystem.stopModules()));
        
        SequentialCommandGroup autonLeftDriveAndDeliver = new SequentialCommandGroup(
                              new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryLeft.getInitialPose())), 
                              new ManualEncoderCalibration(swerveSubsystem),
                              leftDriveAndDeliverCommand,  
                              new InstantCommand(() -> swerveSubsystem.stopModules()));
        

        // Command to Auto Balance                      
        AutonomousDriveCommand autonAutoBalance = new AutonomousDriveCommand(swerveSubsystem, 6);
        SetTo90 setTo90 = new SetTo90(swerveSubsystem, 0.25);

        
      
      //right or left

      // autonChooser.setDefaultOption("Right or Left", scoreHighGoalAuton
      //   .andThen(new ArmGoToHigh2(arm))
      //   .andThen(retractArmAuton)
      //   .andThen(Commands.parallel(autonForwardPath, new AutonIntakeCommand(intake, 6))));

      //auton Balance
      // autonChooser.setDefaultOption("Right or Left", (new InstantCommand(()-> swerveSubsystem.resetOdometry(trajectory3.getInitialPose()))
      //   .andThen(Commands.parallel(autonForwardPath, new AutonIntakeCommand(intake, 6)))));

      //Right path, delivers and drives out of community(Tested)
      autonChooser.setDefaultOption("Right Drive And Deliver", scoreHighGoalAuton.andThen(retractArmAuton).andThen(Commands.parallel(autonForwardPath, new AutonIntakeCommand(intake, 8))));
      //Left path, Deliver and drive out of community(Not Tested)
      autonChooser.addOption("Left Drive and Deliver", new InstantCommand(()->arm.resetArmEncoder())
      .andThen(new ZeroExtender(extender))
      .andThen(new ClawGrabCone(claw))
      .andThen(new ArmGoToHighMotionMagic(arm))
      .andThen(new ExtendHighGoal(extender, 2.0))
      .andThen(new ArmGoToHigh2(arm))
      .andThen(new WaitCommand(0.5))
      .andThen(new ClawReset(claw))
      .andThen(new ExtenderRetractToZero(extender))
      .andThen(new InstantCommand(()->arm.armMoveToZeroPosition()))
      .andThen(Commands.parallel(autonLeftDriveAndDeliver, new AutonIntakeCommand(intake, 8))));
      //autonChooser.addOption("Left Forward Path", scoreHighGoalAuton.andThen(retractArmAuton).andThen(Commands.parallel(autonForwardLeftPath, new AutonIntakeCommand(intake, 8))));
      //autonChooser.setDefaultOption("Right Forward Path", scoreHighGoalAuton.andThen(retractArmAuton));
      
      //Left path, only drives(Not Tested)
      autonChooser.addOption("Left Drive Path", Commands.parallel(autonForwardLeftPath, new AutonIntakeCommand(intake, 8)));
      //Delivers the cone alone(NOT Tested)
      autonChooser.addOption("Deliver Routine", scoreHighGoalDeliverAuton.andThen(retractArmDeliverAuton));


      autonChooser.addOption("AutoBalance Routine", autonDriveToPad.andThen(autonAutoBalance).andThen(setTo90));
      

      // autonChooser.addOption("Auton Balance", autonForwardPath
      //   .andThen(autonomousDriveCommand).andThen(setTo90));

      SmartDashboard.putData(autonChooser);

}
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      //Arm Buttons
      secondaryController.pov(0).whileTrue(new InstantCommand(()->arm.armUp()));
      secondaryController.pov(-1).whileTrue(new InstantCommand(()->arm.stopArm()));
      secondaryController.pov(180).whileTrue(new InstantCommand(()->arm.armDown()));
      // secondaryController.pov(270).onTrue(new InstantCommand(()->arm.armHighGoal()))
      //   .onFalse(new InstantCommand(()->arm.stopArm()));
      // secondaryController.pov(90).onTrue(new InstantCommand(()->arm.armMidGoal()))
      //   .onFalse(new InstantCommand(()->arm.stopArm()));

      // secondaryController.pov(45).whileTrue(new InstantCommand(()->arm.armUp()));
      // secondaryController.pov(315).whileTrue(new InstantCommand(()->arm.armUp()));
      // secondaryController.pov(225).whileTrue(new InstantCommand(()->arm.armDown()));
      // secondaryController.pov(135).whileTrue(new InstantCommand(()->arm.armDown()));
      // secondaryController.rightTrigger().whileTrue(new InstantCommand(()->arm.armMidGoal()))
      //   .whileFalse(new InstantCommand(()->arm.stopArm()));
      
      secondaryController.start().onTrue(new InstantCommand(()->arm.armMoveToZeroPosition()));
      //   .onFalse(new InstantCommand(()->arm.stopArm()));


      //Extender Buttons
      secondaryController.b().onTrue(new InstantCommand(()->extender.extend()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));      
      secondaryController.x().onTrue(new InstantCommand(()->extender.retract()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));

      




      //Manual Orienter Button
      // Trigger orienterTrigger = new Trigger(()->secondaryController.getRawAxis(3)<-0.15);
      // orienterTrigger.onTrue(new InstantCommand(()->orienter.microwaveManualSpin()));
      // orienterTrigger.onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      // Trigger orienterTrigger2 = new Trigger(()->secondaryController.getRawAxis(3)>0.15);
      // orienterTrigger2.onTrue(new InstantCommand(()->orienter.microwaveReverseManualSpin()));
      // orienterTrigger2.onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      secondaryController.pov(90).onTrue(new InstantCommand(()->orienter.microwaveManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      secondaryController.pov(270).onTrue(new InstantCommand(()->orienter.microwaveReverseManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      

      //Claw Buttons
      secondaryController.rightBumper().onTrue(new InstantCommand(()->claw.clawRelease()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
      secondaryController.leftBumper().onTrue(new InstantCommand(()->claw.clawGrab()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
        // Trigger clawTrigger = new Trigger(()->secondaryController.getRawAxis(2)<-0.15);
      // clawTrigger.onTrue(new InstantCommand(()->claw.clawGrab()));
      // clawTrigger.onFalse(new InstantCommand(()->claw.stopGrab()));

      // .onFalse(new InstantCommand(()->claw.stopGrab()));
      // secondaryController.rightTrigger().whileTrue(new InstantCommand(()->claw.clawGrab()))
        // .onFalse(new InstantCommand(()->claw.stopGrab()));
      
      
      //Intake Buttons
      primaryController.leftBumper().onTrue(new InstantCommand(()->intake.intakeFeedIn()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      primaryController.rightBumper().onTrue(new InstantCommand(()->intake.intakeFeedOut()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      
      
      // if(secondaryController.getRightX()<-0.15){
      //   new InstantCommand(()->orienter.microwaveManualSpin());
      // }else if(secondaryController.getRightX()>0.15){
      //   new InstantCommand(()->orienter.microwaveReverseManualSpin());
      // }else{
      //   new InstantCommand(()->orienter.stopMicrowave());
      // }
      // secondaryController.leftTrigger().whileTrue(new InstantCommand(() -> orienter.microwaveManualSpin()))
      //   .whileFalse(new InstantCommand(()->orienter.stopMicrowave()));
      // secondaryController.rightTrigger().whileTrue(new InstantCommand(() -> orienter.microwaveReverseManualSpin()))
      //   .whileFalse(new InstantCommand(()->orienter.stopMicrowave()));
      

      //Auto command groups
      // secondaryController.pov(90).onTrue(new SequentialCommandGroup(
      //   new InstantCommand(()->extender.gotoDefaultPos()),
      //   new ClawGrabCone(claw),
      //   new ArmGoToMid(arm),
      //   new InstantCommand(()->  
      //   {
      //     System.out.print("EXTENDER MID GOAL!@#@!@#$$%^");
      //     extender.extendMidGoal();})
      // ));
      // secondaryController.pov(270).onTrue(new SequentialCommandGroup(
      //   new InstantCommand(()->extender.gotoDefaultPos()),
      //   new ClawGrabCone(claw),
      //   new ArmGoToHigh(arm),
      //   new ExtendHighGoal(extender, 2)
      //   // new InstantCommand(()->
      //   // {
      //   //   System.out.println("Extender High Goal");
      //   //   extender.extendHighGoal();})
      // ));



      secondaryController.back().onTrue(scoreHighGoal);
      secondaryController.rightStick().onTrue(retractArm);


      //new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

      return autonChooser.getSelected();
              
  }

  }
  