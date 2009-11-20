package team153;

import battlecode.common.*;
import static battlecode.common.GameConstants.*;
import java.util.*;

public class ChannelerPlayer extends AttackPlayer {

    public ChannelerPlayer(RobotController controller) {
        super(controller);
        maxDistanceAway = 2;
    }

    public void run() {
        myTeam = controller.getTeam();

        while(true) {
            int startTurn = Clock.getRoundNum();
            controller.setIndicatorString(0, controller.getLocation().toString());
            parseMessages();
            if(isEnergonLow()) {
                while(!isEnergonFull()) {
                    requestEnergonTransfer();
                    controller.yield();
                }
                continue;
            }
            
            processEnemies();
            if(enemies.size() > 0) {
                sortEnemies();
                if(inRangeEnemies.size() == 0 && outOfRangeEnemies.size() > 0 && controller.getRoundsUntilMovementIdle() == 0 && getDistanceToNearestArchon() <= maxDistanceAway) {
                    EnemyInfo closest = getCheapestEnemy(outOfRangeEnemies);
                    movingToAttack = true;
                    moveOnceTowardsLocation(closest.location);
                    movingToAttack = false;
                }
                try {
                    controller.drain();
                } catch (Exception e) {}
            }

            if(startTurn == Clock.getRoundNum() || controller.hasActionSet())
                controller.yield();
        }
    }
}
