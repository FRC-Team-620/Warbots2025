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
    timer.reset();
    intake.set(Constants.IntakeConstants.kCoralIntakeSpeedDutyCycle / 2.0);
  }

  @Override
  public void execute() {
    // Once branch 178 is merged into master, use the isCoralInIntake method instead of distance.
    // this will not work perfectly consistently
    if (intake.getCoralDistance() > 20) {
      timer.start();
    }
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return timer.get() > 0.4;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
  }
}
