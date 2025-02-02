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

  public Trigger turboMode() {
    return driver.leftBumper();
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

  public Trigger intakeCoral() {
    return operator.leftTrigger();
  }

  public Trigger extakeCoral() {
    return operator.rightTrigger();
  }

  public Trigger placeCoralL1() {
    return operator.a();
  }

  public Trigger placeCoralL2() {
    return operator.b();
  }

  public Trigger placeCoralL3() {
    return operator.x();
  }

  public Trigger placeCoralL4() {
    return operator.y();
  }

  public Trigger removeAlgaeL23() {
    return operator.leftBumper();
  }

  public Trigger removeAlgaeL34() {
    return operator.rightBumper();
  }

  public Trigger scoreProcessor() {
    return operator.back();
  }

  public Trigger scoreBarge() {
    return operator.start();
  }

  public Trigger climbUp() {
    return operator.povUp();
  }

  public Trigger climbDown() {
    return operator.povDown();
  }

  public Trigger indexerUp() {
    return operator.leftStick();
  }

  public Trigger indexerDown() {
    return operator.rightStick();
  }
}
