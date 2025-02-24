package org.jmhsrobotics.frc2025.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class RedLEDCommand extends Command {
  private LED led;
  private final LEDPattern red = LEDPattern.solid(Color.kRed);

  public RedLEDCommand(LED led) {
    this.led = led;
    addRequirements(this.led);
  }

  @Override
  public void execute() {
    led.setPattern(red);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
