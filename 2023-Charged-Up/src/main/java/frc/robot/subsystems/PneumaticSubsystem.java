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
  DoubleSolenoid intakeDep;

  public PneumaticSubsystem() {
    clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.CLAW_CHANNEL);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.INTAKE_FORWARD_CHANNEL, Constants.Pneumatics.INTAKE_REVERSE_CHANNEL);
    intakeDep = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.INTAKE_DEPLOY_FORWARD_CHANNEL, Constants.Pneumatics.INTAKE_DEPLOY_REVERSE_CHANNEL);
    intakeSolenoid.set(kOff);
    intakeDep.set(kOff);
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

  public void turnOffIntake()
  {
    intakeDep.set(kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
