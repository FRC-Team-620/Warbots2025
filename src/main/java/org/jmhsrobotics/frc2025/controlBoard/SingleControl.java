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

  // =======Operator Controls=======

  public Trigger intakeCoral() {
    return driver.leftTrigger();
  }

  public Trigger extakeCoral() {
    return driver.rightTrigger();
  }

  public Trigger placeCoralLevel1() {
    return driver.a();
  }

  public Trigger placeCoralLevel2() {
    return driver.b();
  }

  public Trigger placeCoralLevel3() {
    return driver.x();
  }

  public Trigger placeCoralLevel4() {
    return driver.y();
  }

  public Trigger scoreAlgaeProcesser() {
    return driver.a();
  }

  public Trigger scoreAlgaeBarge() {
    return driver.y();
  }

  public Trigger elevatorIntakeCoral() {
    return driver.a();
  }

  public Trigger takeAlgaeLevel2() {
    return driver.b();
  }

  public Trigger takeAlgaeLevel3() {
    return driver.x();
  }

  public Trigger takeAlgaeQTip() {
    return driver.y();
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

  public Trigger changeModeLeft() {
    return driver.back();
  }

  public Trigger changeModeRight() {
    return driver.start();
  }
}
