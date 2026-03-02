package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class HubShiftUtil {
    public static Optional<Time> getTimeInCurrentShift() {
        Optional<HubShift> currentHubShift = getCurrentHubShift();

        if (currentHubShift.isPresent()) {
            int elapsedTime = getMatchElapsedTime().get();

            return Optional.of(Seconds.of(
                currentHubShift.get().endTimeElapsed - elapsedTime
            ));
        }

        return Optional.empty();
    }

    public static boolean isHubActive() {
        Optional<HubShift> currentHubShift = getCurrentHubShift();

        if (currentHubShift.isPresent()) {
            Alliance autoWinner = getAutoWinner().orElse(FieldUtils.getAlliance());

            if (autoWinner == FieldUtils.getAlliance()) {
                return currentHubShift.get().activeHub == ActiveHub.AUTO_WINNER;
            } else {
                return currentHubShift.get().activeHub == ActiveHub.AUTO_LOOSER;
            }
        } else {
            return DriverStation.isEnabled();
        }
    }

    public static Optional<HubShift> getCurrentHubShift() {
        Optional<Integer> elapsedTime = getMatchElapsedTime();

        if (elapsedTime.isPresent()) {
            int elapsedTimeValue = elapsedTime.get();

            for (HubShift hubShift : HubShift.values()) {
                if (elapsedTimeValue <= hubShift.endTimeElapsed) {
                    return Optional.of(hubShift);
                }
            }
        }

        return Optional.empty();
    }

    public static Optional<Integer> getMatchElapsedTime() {
        double periodTime = DriverStation.getMatchTime();

        if (DriverStation.isDisabled()) {
            return Optional.empty();
        } else if (DriverStation.isAutonomous()) {
            return Optional.of((int)Math.floor(20.0 - periodTime));
        } else {
            return Optional.of((int)Math.floor(140.0 - periodTime + 20.0));
        }
    }
    public static Optional<Alliance> getAutoWinner() {
        String gameData = DriverStation.getGameSpecificMessage();
        char autoWinner = (gameData.length() > 0) ? gameData.charAt(0) : ' ';

        return switch (autoWinner) {
            case 'B' -> Optional.of(Alliance.Blue);
            case 'R' -> Optional.of(Alliance.Red);
            default -> Optional.empty();
        };
    }

    public static enum HubShift {
        AUTO(0, 20, ActiveHub.BOTH),
        TRANSITION(20, 30, ActiveHub.BOTH),
        SHIFT_1(30, 55, ActiveHub.AUTO_LOOSER),
        SHIFT_2(55, 80, ActiveHub.AUTO_WINNER),
        SHIFT_3(80, 105, ActiveHub.AUTO_LOOSER),
        SHIFT_4(105, 130, ActiveHub.AUTO_WINNER),
        ENDGAME(130, 160, ActiveHub.BOTH);

        public final int startTimeElapsed;
        public final int endTimeElapsed;
        public final ActiveHub activeHub;

        private HubShift(int startTimeElapsed, int endTimeElapsed, ActiveHub activeHub) {
            this.startTimeElapsed = startTimeElapsed;
            this.endTimeElapsed = endTimeElapsed;
            this.activeHub = activeHub;
        }
    }

    public static enum ActiveHub {
        BOTH,
        AUTO_WINNER,
        AUTO_LOOSER;
    }
}
