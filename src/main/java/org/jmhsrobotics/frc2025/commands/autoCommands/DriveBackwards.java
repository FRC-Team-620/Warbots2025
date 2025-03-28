package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;

public class DriveBackwards extends Command {
  private Drive drive;

  public DriveBackwards(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.runVelocity(new ChassisSpeeds(-0.75, 0, 0));
  }

  @Override
  public void execute() {
    drive.runVelocity(new ChassisSpeeds(-0.75, 0, 0));
  }

  @Override
  public boolean isFinished() {
    drive.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
