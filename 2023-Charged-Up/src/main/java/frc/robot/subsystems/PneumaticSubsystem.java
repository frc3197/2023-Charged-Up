package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PneumaticSubsystem extends SubsystemBase {
  
  DoubleSolenoid solenoid;

  public PneumaticSubsystem() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.FORWARD_CHANNEL, Constants.Pneumatics.REVERSE_CHANNEL);
    solenoid.set(kReverse);
  }

  public void toggle() {
    solenoid.toggle();
    System.out.println(solenoid.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
