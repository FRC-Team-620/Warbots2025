package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;

public class DoubleControl implements ControlBoard {
  public CommandXboxController driver = new CommandXboxController(0);
  public CommandXboxController operator = new CommandXboxController(1);

  public DoubleControl() {
    ControllerMonitor.addController(this.operator.getHID(), "Operator");
    ControllerMonitor.addController(this.driver.getHID(), "Driver");
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
    return driver.rightBumper();
  }

  @Override
  public Trigger upExample() {
    return driver.x();
  }

  @Override
  public Trigger downExample() {
    return driver.a();
  }

  // =======Operator Controls=======

}
