package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleControl implements ControlBoard {
  XboxController driver = new XboxController(0);

  public double rotation() {
    return driver.getRightX();
  }

  public double translationX() {
    return driver.getLeftX();
  }

  public double translationY() {
    return driver.getLeftY();
  }

  public Trigger resetForward() {
    return new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  }
}
