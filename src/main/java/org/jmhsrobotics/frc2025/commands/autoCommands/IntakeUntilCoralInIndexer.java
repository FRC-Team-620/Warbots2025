package org.jmhsrobotics.frc2025.commands.autoCommands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.Robot;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeUntilCoralInIndexer extends Command {
  private Wrist wrist;
  private Intake intake;
  private Indexer indexer;
  private LED led;

  private LEDPattern blinkPattern = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1));

  public IntakeUntilCoralInIndexer(Wrist wrist, Intake intake, Indexer indexer, LED led) {
    this.wrist = wrist;
    this.intake = intake;
    this.indexer = indexer;
    this.led = led;

    addRequirements(wrist, intake, indexer);
  }

  @Override
  public void initialize() {
    indexer.set(Constants.IndexerConstants.kIndexerSpeedRPM);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);

    led.setPattern(blinkPattern);
  }

  @Override
  public void execute() {
    indexer.set(Constants.IndexerConstants.kIndexerSpeedRPM);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);

    led.setPattern(blinkPattern);
  }

  @Override
  public boolean isFinished() {
    if (Robot.isSimulation()) return false;
    return indexer.hasCoral() || intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.set(Constants.IndexerConstants.kIndexerSpeedDutyCycle);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
  }
}
