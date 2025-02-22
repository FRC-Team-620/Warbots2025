package org.jmhsrobotics.frc2025.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;

public class RainbowLEDCommand extends Command {
  private LED led;
  private final LEDPattern rainbow = LEDPattern.rainbow(255, 255);
  private final LEDPattern scrollingRainbow =
      rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), Constants.ledSpacing);

  public RainbowLEDCommand(LED led) {
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void execute() {
    led.setPattern(scrollingRainbow);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // isFinished() and end() methods will need to be set to any pattern as driver feedback
  //   @Override
  //   public boolean isFinished() {
  //       // TODO Auto-generated method stub
  //       return super.isFinished();
  //   }

  //   @Override
  //   public void end(boolean interrupted) {
  //       // TODO Auto-generated method stub
  //       super.end(interrupted);
  //   }
}
