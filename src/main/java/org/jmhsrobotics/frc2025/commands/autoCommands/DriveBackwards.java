package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.drive.GyroIOInputsAutoLogged;

public class DriveBackwards extends Command {
  private Drive drive;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  public DriveBackwards(Drive drive) {
    this.drive = drive;
    this.gyroInputs = drive.getGyroInputs();
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
    if (gyroInputs.yAcceleration <= 0) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
