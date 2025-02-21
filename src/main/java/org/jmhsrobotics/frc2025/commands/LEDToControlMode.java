package org.jmhsrobotics.frc2025.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;

public class LEDToControlMode extends Command {
  private LED led;
  private Intake intake;

  private final LEDPattern searchModePattern = LEDPattern.rainbow(255, 255);
  private final LEDPattern scrollingRainbow =
      searchModePattern.scrollAtAbsoluteSpeed(
          MetersPerSecond.of(0.1), Constants.LEDConstants.kSpacing);

  private final LEDPattern algaeModePattern = LEDPattern.solid(Color.kGreen);
  private final LEDPattern coralModePattern = LEDPattern.solid(Color.kFuchsia);

  public LEDToControlMode(LED led, Intake intake) {
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void execute() {
    if (intake.getMode() == Constants.ModeConstants.kSearch) led.setPattern(scrollingRainbow);
    else if (intake.getMode() == Constants.ModeConstants.kAlgae) led.setPattern(algaeModePattern);
    else led.setPattern(coralModePattern);
  }
}
