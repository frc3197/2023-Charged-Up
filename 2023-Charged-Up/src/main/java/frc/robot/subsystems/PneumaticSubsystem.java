package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
  Solenoid single;

  public PneumaticSubsystem() {
    single = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
  }

  public void toggleOn() {
    while(!single.get()) {
      single.toggle();
    }
    System.out.println("Turning on: " + single.get());
  }

  public void toggleOff() {
    while(single.get()) {
      single.toggle();
    }
    System.out.println("Turning off: " + single.get());
  }

  public void toggleSingle() {
    single.toggle();
  }

  public boolean getToggle() {
    return single.get();
  }

  @Override
  public void periodic() {
  }
}
