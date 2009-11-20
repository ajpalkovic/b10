package team153;

import battlecode.common.*;
import static battlecode.common.GameConstants.*;
import java.util.*;

public class CannonPlayer extends AttackPlayer {

    public CannonPlayer(RobotController controller) {
        super(controller);
        
    }

    public void run() {
        myTeam = controller.getTeam();
        sendNewUnit();
        while(true) {
            int startTurn = Clock.getRoundNum();
            autoTransferEnergonBetweenUnits();
            controller.setIndicatorString(0, controller.getLocation().toString());
            processEnemies();

            if(isEnergonLow()) {
                while(!isEnergonFull()) {
                    requestEnergonTransfer();
                    controller.yield();
                }
                continue;
            }

            sortEnemies();
            EnemyInfo enemy = selectEnemy();
            if(enemy != null) {
                // attack
                if (!controller.canAttackSquare(enemy.location)) {
                    faceLocation(enemy.location);
                    processEnemies();
                }
                executeAttack(enemy.location, enemy.type.isAirborne() ? RobotLevel.IN_AIR : RobotLevel.ON_GROUND);
                processEnemies();
                attackLocation = enemy.location;
            } else {
                if(outOfRangeEnemies.size() > 0 || outOfRangeArchonEnemies.size() > 0) {
                    // only move if we can do it in 1 turn or less
                    if(controller.getRoundsUntilMovementIdle() == 0)
                        moveToAttack();
                } else {
                    MapLocation archon = findNearestArchon();
                    if(archon != null && !controller.getLocation().isAdjacentTo(archon) && controller.getRoundsUntilMovementIdle() < 2)
                        moveOnceTowardsLocation(archon);
                }
            }
            
            switch (currentGoal)
            {
            	case Goal.followingArchon:
            		for (Robot r : controller.senseNearbyAirRobots())
            			if (r.getID() == followingArchonNumber)
            				try{
            				goByBugging(map.getNotNull(controller.senseRobotInfo(r).location));
            				}catch (Exception e)
            				{ p("----------------cannot sense robot info in following archon");}
            				
            	break;            	
            }

            if(startTurn == Clock.getRoundNum() || controller.hasActionSet())
                controller.yield();
        }
    }
}
