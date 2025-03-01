package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class ScoreCoral extends Command {
  private Intake intake;
  private Timer timer = new Timer();

  public ScoreCoral(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    timer.restart();
    intake.set(Constants.IntakeConstants.kCoralIntakeSpeedDutyCycle / 2.0);
  }

  @Override
  public void execute() {
    // Once branch 178 is merged into master, use the isCoralInIntake method instead of distance.
    // this will not work perfectly consistently
    intake.set(Constants.IntakeConstants.kCoralIntakeSpeedDutyCycle / 2.0);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
  }
}
