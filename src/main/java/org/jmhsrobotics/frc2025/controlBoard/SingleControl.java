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
    return nop;
  }

  public Trigger intakeAlgae() {
    return nop;
  }

  public Trigger placeCoralL1() {
    return nop;
  }

  public Trigger placeCoralL2() {
    return nop;
  }

  public Trigger placeCoralL3() {
    return nop;
  }

  public Trigger placeCoralL4() {
    return nop;
  }

  public Trigger removeAlgaeL23() {
    return nop;
  }

  public Trigger removeAlgaeL34() {
    return nop;
  }

  public Trigger scoreProcessor() {
    return nop;
  }

  public Trigger scoreBarge() {
    return nop;
  }

  public Trigger climbUp() {
    return nop;
  }

  public Trigger climbDown() {
    return nop;
  }

  public Trigger indexerUp() {
    return nop;
  }

  public Trigger indexerDown() {
    return nop;
  }
}
