package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.HubShiftUtil.HubShift;

public final class ShiftTimeUtil {
    public static double getAllianceShiftTime() {
        if (DriverStation.isTeleopEnabled()) {
            if (!HubShiftUtil.isHubActive()) {
                return -HubShiftUtil.getTimeInCurrentShift().orElse(Seconds.zero()).in(Seconds);
            } else {
                Optional<HubShift> nextShift = HubShiftUtil.getCurrentHubShift().orElse(HubShift.AUTO).getNext();
                boolean wonAuto = HubShiftUtil.getAutoWinner().orElse(FieldUtils.getAlliance()) == FieldUtils.getAlliance();
                int nextShiftAdjustment = 0;

                if (nextShift.isPresent() && nextShift.get().isActive(wonAuto)) {
                    nextShiftAdjustment = (nextShift.get().endTimeElapsed - nextShift.get().startTimeElapsed);
                }

                return HubShiftUtil.getTimeInCurrentShift().orElse(Seconds.zero()).in(Seconds) + nextShiftAdjustment;
            }
        } else {
            return 0.0;
        }
    }
}
