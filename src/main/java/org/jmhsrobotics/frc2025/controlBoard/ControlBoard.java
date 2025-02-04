package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {

  // ========Driver Controls========
  public double rotation();

  public double translationX();

  public double translationY();

  public Trigger resetForward();

  public Trigger turboMode();

  public Trigger upExample();

  public Trigger downExample();

  // =======Operator Controls=======

  public Trigger intakeCoral();

  public Trigger extakeCoral();

  public Trigger buttonA();

  public Trigger buttonB();

  public Trigger buttonX();

  public Trigger buttonY();

  public Trigger climbUp();

  public Trigger climbDown();

  public Trigger indexerUp();

  public Trigger indexerDown();

  public Trigger changeModeLeft();

  public Trigger changeModeRight();
}
