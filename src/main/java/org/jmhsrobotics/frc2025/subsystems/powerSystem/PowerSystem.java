package org.jmhsrobotics.frc2025.subsystems.powerSystem;

import au.grapplerobotics.MitoCANdria;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2025.Constants;
import org.littletonrobotics.junction.Logger;

public class PowerSystem extends SubsystemBase {
  PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev); // TODO: set can id
  MitoCANdria powerhouse = new MitoCANdria(Constants.CAN.kMitoCANdriaID);

  public PowerSystem() {
    try {
      powerhouse.setChannelVoltage(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ, 0);

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  private void logChannel(int channel, String name) {
    try {
      powerhouse
          .getChannelCurrent(channel)
          .ifPresentOrElse(
              current -> {
                Logger.recordOutput("mito/" + name + "/currentAmps", current);
              },
              () -> System.out.println("Couldn't get mito-amps: " + channel + " " + name));
      powerhouse
          .getChannelVoltage(channel)
          .ifPresentOrElse(
              current -> {
                Logger.recordOutput("mito/" + name + "/voltage", current);
              },
              () -> System.out.println("Couldn't get mito-volts: " + channel + " " + name));
    } catch (Exception e) {
      System.out.println("Couldn't get mito: " + channel + " " + name);
    }
  }

  @Override
  public void periodic() {
    logChannel(MitoCANdria.MITOCANDRIA_CHANNEL_5VA, "5VA");
    logChannel(MitoCANdria.MITOCANDRIA_CHANNEL_5VB, "5VB");
    logChannel(MitoCANdria.MITOCANDRIA_CHANNEL_ADJ, "ADJ");
    logChannel(MitoCANdria.MITOCANDRIA_CHANNEL_USB1, "USB1");
    logChannel(MitoCANdria.MITOCANDRIA_CHANNEL_USB2, "USB2");
  }
}
