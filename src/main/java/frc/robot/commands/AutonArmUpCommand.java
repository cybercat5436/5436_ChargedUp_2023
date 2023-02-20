package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutonArmUpCommand extends CommandBase {

private Timer timer;
private double timeLimit = 0.0;
private int encoderCounts = 0;
private Arm arm;

public AutonArmUpCommand(Arm arm, int encoderCounts, double time){
    this.encoderCounts = encoderCounts;
    this.timeLimit = time;
    this.arm = arm;
    timer = new Timer();
}
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
     timer.reset();
     timer.start();
     arm.resetArmEncoder();
 }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
     arm.armUp();
     
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
     arm.stopArm();
 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
     
   return (timer.get() > timeLimit ? true : false) || (arm.getArmPosition() >= encoderCounts);
}
}

