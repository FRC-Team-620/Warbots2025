package org.jmhsrobotics.frc2025.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;

public class LED extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private LEDPattern pattern = LEDPattern.solid(Color.kRed);

  public LED() {
    led = new AddressableLED(Constants.LEDConstants.kPWMHeader);
    ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLength);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public LEDPattern getCurrentPattern() {
    return this.pattern;
  }

  public void setPattern(LEDPattern pattern) {
    this.pattern = pattern;
  }
}
