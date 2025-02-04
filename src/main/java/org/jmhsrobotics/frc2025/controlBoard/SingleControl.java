package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;

public class SingleControl implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);

  private Trigger nop =
      new Trigger(
          () -> {
            return false;
          });

  public SingleControl() {
    ControllerMonitor.addController(driver.getHID(), "Driver");
  }

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
    return nop;
  }

  @Override
  public Trigger upExample() {
    return driver.x();
  }

  @Override
  public Trigger downExample() {
    return driver.a();
  }

  public Trigger intakeCoral() {
    return driver.leftTrigger();
  }

  public Trigger extakeCoral() {
    return driver.rightTrigger();
  }

  public Trigger placeCoralL1() {
    return driver.a();
  }

  public Trigger placeCoralL2() {
    return driver.b();
  }

  public Trigger placeCoralL3() {
    return driver.x();
  }

  public Trigger placeCoralL4() {
    return driver.y();
  }

  public Trigger removeAlgaeL23() {
    return driver.leftBumper();
  }

  public Trigger removeAlgaeL34() {
    return driver.rightBumper();
  }

  public Trigger scoreProcessor() {
    return driver.back();
  }

  public Trigger scoreBarge() {
    return driver.start();
  }

  public Trigger climbUp() {
    return driver.povUp();
  }

  public Trigger climbDown() {
    return driver.povDown();
  }

  public Trigger indexerUp() {
    return driver.leftStick();
  }

  public Trigger indexerDown() {
    return driver.rightStick();
  }
}
