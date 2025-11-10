package org.firstinspires.ftc.teamcode.lib;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class Commands {

    public class SpinChute extends CommandBase {
        private final Hardware.Chute chute;
        private final boolean isReversed;

        public SpinChute(Hardware.Chute subsystem, boolean reverse) {
            chute = subsystem;
            isReversed = reverse;
            addRequirements(chute);
        }
        @Override
        public void initialize() {
            if (isReversed) {
                chute.reverse();
            } else {
                chute.start();
            }
        }
        @Override
        public void end(boolean interrupted) {
            chute.stop();
        }
    }
    public class SpinIntake extends CommandBase {
        private final Hardware.Intake intake;
        private final boolean isReversed;

        public SpinIntake(Hardware.Intake subsystem, boolean reverse) {
            intake = subsystem;
            isReversed = reverse;
            addRequirements(intake);
        }
        @Override
        public void initialize() {
            if (isReversed) {
                intake.reverse();
            } else {
                intake.start();
            }
        }
        @Override
        public void end(boolean interrupted) {
            intake.stop();
        }
    }

    public class SetShooterVelocity extends CommandBase {
        private final Hardware.Shooter shooter;
        private final double targetVelocity;

        public SetShooterVelocity(Hardware.Shooter subsystem, double targetVelocity) {
            shooter = subsystem;
            this.targetVelocity = targetVelocity;
            addRequirements(shooter);
        }
        @Override
        public void initialize() {
            shooter.setLauncherVelocity(targetVelocity);
        }
        @Override
        public void execute() {
            shooter.keepLauncherAtVelocity();
        }
        @Override
        public boolean isFinished() {
            return shooter.isLauncherAtVelocity();
        }
        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.stop();
            }
        }
    }
    public class OpenLock extends CommandBase {
        private final Hardware.Lock lock;
        public OpenLock(Hardware.Lock subsystem) {
            lock = subsystem;
            addRequirements(lock);
        }
        @Override
        public void initialize() {
            lock.open();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }
    public class CloseLock extends CommandBase {
        private final Hardware.Lock lock;
        public CloseLock(Hardware.Lock subsystem) {
            lock = subsystem;
            addRequirements(lock);
        }
        @Override
        public void initialize() {
            lock.close();
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class DetectColorAndSort extends CommandBase {
        private final Hardware.IntakeSensor cameraFront;
        private final Hardware.IntakeSensor cameraBack;
        private final Hardware.Sorter sorter;
        public DetectColorAndSort(Hardware.IntakeSensor subsystem1, Hardware.IntakeSensor subsystem2, Hardware.Sorter subsystem3) {
            cameraFront = subsystem1;
            cameraBack = subsystem2;
            sorter = subsystem3;
            addRequirements(cameraFront, cameraBack, sorter);
        }
        @Override
        public void initialize() {
            Hardware.ArtifactType backDetected = cameraBack.detectColor();
            Hardware.ArtifactType frontDetected = cameraFront.detectColor();

            if (backDetected != Hardware.ArtifactType.NONE) {
                sorter.updateServo(backDetected, true);
            } else if (frontDetected != Hardware.ArtifactType.NONE) {
                sorter.updateServo(backDetected, false);
            } else {
                //add none here later
            }
        }
        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class RunIntake extends ParallelCommandGroup {
        public RunIntake(Hardware.Chute chuteRight, Hardware.Chute chuteLeft, Hardware.Intake intakeFront, Hardware.Intake intakeBack)
        {
            addCommands(
                    new SpinChute(chuteRight, false),
                    new SpinChute(chuteLeft, false),
                    new SpinIntake(intakeFront, false),
                    new SpinIntake(intakeBack, false)
            );
            addRequirements(chuteLeft, chuteRight, intakeBack, intakeFront);
        }

    }

    public class Shoot extends SequentialCommandGroup {
        public Shoot(Hardware.Shooter shooter, Hardware.Chute chute, Hardware.Lock lock, double velocity)
        {
            addCommands(
                    new SetShooterVelocity(shooter, velocity),
                    new ParallelCommandGroup(
                            new SetShooterVelocity(shooter, velocity), //keep velocity at target
                            new ParallelDeadlineGroup(
                                    new WaitCommand(5000), //probably replace this with waiting for a dip in velo?
                                    new SpinChute(chute, false),
                                    new OpenLock(lock)
                            )
                    ),
                    new CloseLock(lock)
            );
            addRequirements(shooter, chute, lock);
        }

    }
}
