package org.jmhsrobotics.frc2025.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeFromIndexer extends Command {
  private Wrist wrist;
  private Intake intake;
  private Indexer indexer;
  private LED led;

  private boolean coralIntaked = false;

  private LEDPattern blinkPattern = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1));

  public IntakeFromIndexer(Wrist wrist, Intake intake, Indexer indexer, LED led) {
    this.wrist = wrist;
    this.intake = intake;
    this.indexer = indexer;
    this.led = led;

    addRequirements(wrist, intake, indexer, led);
  }

  @Override
  public void initialize() {
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
    indexer.set(Constants.IndexerConstants.kIndexerSpeedRPM);

    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);

    this.coralIntaked = false;
    led.setPattern(blinkPattern);
  }

  @Override
  public void execute() {
    led.setPattern(blinkPattern);

    if (intake.isCoralInIntake()) {
      intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed);
      coralIntaked = true;
    } else {
      intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
      indexer.set(Constants.IndexerConstants.kIndexerSpeedRPM);
    }
  }

  @Override
  public boolean isFinished() {
    // return intake.isCoralInIntake();
    return coralIntaked && !intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    indexer.set(0);
    wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
    System.out.println("Intake From Indexer Complete: " + interrupted);
  }
}
