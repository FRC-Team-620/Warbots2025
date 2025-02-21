package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.led.LED;

public class LEDFlashPattern extends Command {
  private LED led;
  private double duration;
  private double interval = 1.0 / Constants.LEDConstants.kFlashFrequency;

  private final LEDPattern firstPattern;
  private final LEDPattern secondPattern;

  private Timer timer = new Timer();

  public LEDFlashPattern(
      LED led, LEDPattern firstPattern, LEDPattern secondPattern, double duration) {
    this.led = led;
    this.duration = duration;
    this.firstPattern = firstPattern;
    this.secondPattern = secondPattern;
  }

  @Override
  public void initialize() {
    timer.restart();
    led.setPattern(firstPattern);
  }

  @Override
  public void execute() {
    // Changes the pattern applied if the timer value is divisible by time that one pattern stays,
    // and then changes the pattern to the other color
    if (timer.get() % interval == 0) {
      if (led.getCurrentPattern() == firstPattern) led.setPattern(secondPattern);
      else led.setPattern(firstPattern);
    }
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return timer.get() > this.duration;
  }
}
