package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class States {
    public static class Shooter {
        public enum FlywheelStates {
            Frozen, Varying;
        }
        
        public enum HoodState {
            Frozen, Varying;
        }
    }

    public static class Intake {
        public enum PivotState {
            HihglyStowed(-115), Medium(-55), FullDeploy(0);

            public Angle angle;

            private PivotState(Angle angle) {
                this.angle = angle;
            }

            private PivotState(float angle) {
                this.angle = Degrees.of(angle);
            }
        }

        public enum RollerState {
            On(3000), Reverse(-3000), Off(0);

            public AngularVelocity speed;

            private RollerState(AngularVelocity speed) {
                this.speed = speed;
            }

            private RollerState(float speed) {
                this.speed = RPM.of(speed);
            }
        }
    }

    public static class Indexer {
        public enum TripleRollerStates {
            On(2377, 4414, 4414), Off(0, 0, 0);
            
            public AngularVelocity floorSpeed;
            public AngularVelocity ceilingSpeed;
            public AngularVelocity kickerSpeed;

            private TripleRollerStates(AngularVelocity floorSpeed,
                AngularVelocity ceilingSpeed, 
                AngularVelocity kickerSpeed) {
                    this.floorSpeed = floorSpeed;
                    this.ceilingSpeed = ceilingSpeed;
                    this.kickerSpeed = kickerSpeed;
            }

            private TripleRollerStates(double floorSpeed, double ceilingSpeed, double kickerSpeed) {
                this.floorSpeed = RPM.of(floorSpeed);
                this.ceilingSpeed = RPM.of(ceilingSpeed);
                this.kickerSpeed = RPM.of(kickerSpeed);
            }
        }
    }
}
