package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;

/* This command is used in auto to move the bot forward across the auto point line*/

public class DriveTimeCommand extends Command {
  private Drive drive;

  private double driveSeconds;
  private ChassisSpeeds speeds;

  private Timer timer = new Timer();

  public DriveTimeCommand(double seconds, double velocityMPS, Drive subsystem) {

    driveSeconds = seconds;
    speeds = new ChassisSpeeds(velocityMPS, 0, 0);

    drive = subsystem;
    addRequirements(subsystem);
  }

  public void initialize() {

    timer.start();
    timer.reset();
  }

  @Override
  public void execute() {

    this.drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {

    return timer.hasElapsed(driveSeconds);
  }

  public void end() {
    this.drive.stop();
  }
}
