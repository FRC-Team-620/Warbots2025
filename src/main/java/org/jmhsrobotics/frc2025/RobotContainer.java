// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2025;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.jmhsrobotics.frc2025.commands.DriveCommands;
import org.jmhsrobotics.frc2025.commands.ElevatorCommand;
import org.jmhsrobotics.frc2025.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2025.controlBoard.SingleControl;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.drive.GyroIO;
import org.jmhsrobotics.frc2025.subsystems.drive.GyroIOPigeon2;
import org.jmhsrobotics.frc2025.subsystems.drive.swerve.ModuleIO;
import org.jmhsrobotics.frc2025.subsystems.drive.swerve.ModuleIOSimRev;
import org.jmhsrobotics.frc2025.subsystems.drive.swerve.ModuleIOThrifty;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.elevator.ElevatorIO;
import org.jmhsrobotics.frc2025.subsystems.elevator.SimElevatorIO;
import org.jmhsrobotics.frc2025.subsystems.elevator.VortexElevatorIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Elevator elevator;

  private final ControlBoard control;

  private final ElevatorCommand up;
  private final ElevatorCommand down;
  // Controller

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Change to "SingleControl" or "DoubleControl" here based on preference
    this.control = new SingleControl();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOThrifty(0),
                new ModuleIOThrifty(1),
                new ModuleIOThrifty(2),
                new ModuleIOThrifty(3));

        elevator = new Elevator(new VortexElevatorIO() {});
        System.out.println("Mode: REAL");
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSimRev(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev());

        elevator = new Elevator(new SimElevatorIO() {});
        System.out.println("Mode: SIM");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        elevator = new Elevator(new ElevatorIO() {});
        System.out.println("Mode: DEFAULT");
        break;
    }
    up = new ElevatorCommand(this.elevator, .75);
    down = new ElevatorCommand(this.elevator, .25);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    setupSmartDashbaord();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -control.translationY(),
            () -> -control.translationX(),
            () -> -control.rotation()));

    // Reset gyro to 0° when right bumper is pressed
    control
        .resetForward()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive));
  }

  private void setupSmartDashbaord() {
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

    SmartDashboard.putData("up", up);
    SmartDashboard.putData("down", down);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
