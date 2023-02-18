package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PneumaticSubsystem extends SubsystemBase {
  
  //DoubleSolenoid clawSolenoid;
  Solenoid clawSolenoid;
  DoubleSolenoid intakeSolenoid;

  public PneumaticSubsystem() {
    clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.CLAW_CHANNEL);
    //clawSolenoid.set(kReverse);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.INTAKE_FORWARD_CHANNEL, Constants.Pneumatics.INTAKE_REVERSE_CHANNEL);
    intakeSolenoid.set(kReverse);
  }

  public void toggleClaw() {
    clawSolenoid.toggle();
  }

  public void closeClaw() {
    /*if(clawSolenoid.get() == kForward) {
      clawSolenoid.toggle();
    }*/
  }

  public void openClaw() {
    /*if(clawSolenoid.get() == kReverse) {
      clawSolenoid.toggle();
    }*/
  }

  public void toggleIntake() {
    intakeSolenoid.toggle();
  }

  public void lowerIntake() {
    if(intakeSolenoid.get() == kForward) {
      intakeSolenoid.toggle();
    }
  }

  public void raiseIntake() {
    if(intakeSolenoid.get() == kReverse) {
      intakeSolenoid.toggle();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
