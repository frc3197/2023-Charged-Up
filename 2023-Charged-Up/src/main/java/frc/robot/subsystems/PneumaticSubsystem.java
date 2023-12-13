package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;

public class PneumaticSubsystem extends SubsystemBase {

  // DoubleSolenoid clawSolenoid;
  Solenoid clawSolenoid;
  DoubleSolenoid intakeSolenoid;
  DoubleSolenoid intakeDep;
  DoubleSolenoid armWrist;
  Compressor compressor;

  boolean wasClosed = true;
  boolean firstZone = true;

  int numTriggers = 0;

  public PneumaticSubsystem() {
    clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.CLAW_CHANNEL);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    armWrist = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
    armWrist.set(kForward);
  }

  public void toggleWrist() {
    armWrist.toggle();
  }

  public void autoWristOn() {
    if(armWrist.get() == kForward) {
      armWrist.toggle();
    }
  }

  public void autoWristOff() {
    if(armWrist.get() != kForward) {
      armWrist.toggle();
    }
  }

  public void toggleClaw() {
    clawSolenoid.toggle();
    numTriggers++;
  }

  public void enteringZone() {
    firstZone = true;
  }

  public void closeClaw() {
    if (!clawSolenoid.get()) {
      if (firstZone) {
        wasClosed = false;
      }
      firstZone = false;
      clawSolenoid.toggle();
    }
  }

  public boolean getPrevious() {
    return wasClosed;
  }

  public void openClaw() {
    if (clawSolenoid.get()) {
      clawSolenoid.toggle();
    }
  }

  public void openWrist()
  {
    if(armWrist.get() == kReverse) {
      armWrist.toggle();
    }
  }

  public void closeWrist()
  {
    if (armWrist.get() == kForward) {
      armWrist.toggle();
    }
  }

  public void toggleIntake() {
    intakeSolenoid.toggle();
  }

  public void lowerIntake() {
    if (intakeSolenoid.get() == kForward) {
      intakeSolenoid.toggle();
    }
  }

  public void raiseIntake() {
    if (intakeSolenoid.get() == kReverse) {
      intakeSolenoid.toggle();
    }
  }

  public void turnOffIntake() {
    intakeDep.set(kOff);
  }

  public boolean getCompressor()
  {
    return !compressor.isEnabled();
  }

  public int getTriggers()
  {
    return numTriggers;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
