package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.util.ControllerMonitor;

public class SingleControl implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);
  private Intake intake;

  private Trigger nop =
      new Trigger(
          () -> {
            return false;
          });

  public SingleControl(Intake intake) {
    ControllerMonitor.addController(driver.getHID(), "Driver");
    this.intake = intake;
  }

  private Trigger coralMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kCoral;
          });

  private Trigger algaeMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kAlgae;
          });

  private Trigger searchMode =
      new Trigger(
          () -> {
            return intake.getMode() == Constants.ModeConstants.kSearch;
          });

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
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return driver.a().and(coralMode);
  }

  public Trigger placeCoralLevel2() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return driver.b().and(coralMode);
  }

  public Trigger placeCoralLevel3() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;

    return driver.x().and(coralMode);
  }

  public Trigger placeCoralLevel4() {
    //  if (intake.getMode() != Constants.ModeConstants.kCoral) return nop;
    return driver.y().and(coralMode);
  }

  public Trigger scoreAlgaeProcesser() {
    //  if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;

    return (driver.a().or(driver.b())).and(algaeMode);
  }

  public Trigger scoreAlgaeBarge() {
    //  if (intake.getMode() != Constants.ModeConstants.kAlgae) return nop;

    return (driver.y().or(driver.x())).and(algaeMode);
  }

  public Trigger elevatorIntakeCoral() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return driver.a().and(searchMode);
  }

  public Trigger takeAlgaeLevel2() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return driver.b().and(searchMode);
  }

  public Trigger takeAlgaeLevel3() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return driver.x().and(searchMode);
  }

  public Trigger takeAlgaeQTip() {
    //  if (intake.getMode() != Constants.ModeConstants.kSearch) return nop;

    return driver.y().and(searchMode);
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

  public Trigger resetIndexer() {
    return driver.povLeft();
  }
}
