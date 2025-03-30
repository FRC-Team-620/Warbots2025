package org.jmhsrobotics.frc2025.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeFromIndexer extends Command {
  private Wrist wrist;
  private Intake intake;
  private LED led;

  private LEDPattern blinkPattern = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1));

  public IntakeFromIndexer(Wrist wrist, Intake intake, LED led) {
    this.wrist = wrist;
    this.intake = intake;
    this.led = led;

    addRequirements(wrist, intake, led);
  }

  @Override
  public void initialize() {
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);
    led.setPattern(blinkPattern);
  }

  @Override
  public void execute() {
    led.setPattern(blinkPattern);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    System.out.println("Intake From Indexer Complete: " + interrupted);
  }
}
