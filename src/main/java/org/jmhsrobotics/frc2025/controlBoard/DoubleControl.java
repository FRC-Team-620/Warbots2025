package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;

public class DoubleControl implements ControlBoard {
  public XboxController driver = new XboxController(0);
  public XboxController operator = new XboxController(1);

  public DoubleControl() {
    ControllerMonitor.addController(this.operator, "Operator");
    ControllerMonitor.addController(this.driver, "Driver");
  }

  // ========Driver Controls========

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

  // =======Operator Controls=======

}
