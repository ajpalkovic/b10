package team153;

import battlecode.common.*;
import static battlecode.common.GameConstants.*;
import java.util.*;

public class NovaPlayer {
    /**
     * Various int status codes for the action methods to return
     */
	public final int KEY1 = 1234567;
	public final int KEY2 = 7654321;
    class Status {
        public static final int success = 1, fail = 0;
        public static final int notEnoughEnergon = 2;
        public static final int cantMoveThere = 3, goalBlocked = 4;

        public static final int outOfRange = 10, turnsNotIdle = 11;
    }
    
    static class Goal {
        public static final int exploringForFlux = 2, goingTowardsFlux = 3, goingDirectlyToFlux = 4, gettingCloseToFlux=5;
        public static final int supporttingFluxDeposit = 10, collectingFlux = 11;
        public static final int idle = 20;
        public static final int scouting = 30, followingArchon = 31, alliedUnitRelay = 32, fight = 33;
        public static final int findBlock = 40, foundBlock = 41, goingToSteps = 42; 
        public static String toString(int goal) {
        	switch (goal) {
                case exploringForFlux:
                    return "Exploring for Flux";
                case goingTowardsFlux:
                    return "Going towards Flux";
                case goingDirectlyToFlux:
                    return "Going directly to Flux";
                case gettingCloseToFlux:
                    return "Getting closer to a flux depsoit";
                case supporttingFluxDeposit:
                    return "Supporting Flux Deposit";
                case collectingFlux:
                    return "Collecting Flux";
                case idle:
                    return "Idle";
                case scouting:
                    return "Scouting";
                case followingArchon:
                    return "Following Archon";
                case findBlock:
                	return "Finding Block";
                case foundBlock:
                	return "Found Block";
                case goingToSteps:
                	return "Going to steps";
                case alliedUnitRelay:
                    return "Relaying Allied Unit Locations";
                case fight:
                	return "FIGHT!";
                
        	}
        	return "?";
        }
    }
    
    
    public RobotController controller;
    public Robot robot;
    public MapStore map;
    public int moveStraightDelay, moveDiagonalDelay;
    public int leftWall = Integer.MIN_VALUE, rightWall = Integer.MAX_VALUE, topWall = Integer.MIN_VALUE, bottomWall = Integer.MAX_VALUE;
    public int leftWallBounds = Integer.MAX_VALUE, rightWallBounds = Integer.MIN_VALUE,
            topWallBounds = Integer.MAX_VALUE, bottomWallBounds = Integer.MIN_VALUE;

    public ArrayList<Integer> oldEnemies;
    public ArrayList<EnergonTransferRequest> requests;
    public ArrayList<LowAllyRequest> lowAllyRequests;
    public int lowAllyRequestsTurn = 0;

    public ArrayList<MapLocation> oldLocations;
    public int trackingCount = 0;
    public int currentGoal = 0;
    public int followingArchonNumber = -1;
    public double lowEnergonLevel;
    public RobotCache robotCache;

    /* variables for message processing */
    public ArrayList<Integer> messageInts = new ArrayList<Integer>();
    public ArrayList<String> messageStrings = new ArrayList<String>();
    public ArrayList<MapLocation> messageLocations = new ArrayList<MapLocation>();
    public Team myTeam;

    public boolean isInAir;

    public NovaPlayer(RobotController controller) {        
        this.controller = controller;
        map = new MapStore();
        moveStraightDelay = controller.getRobotType().moveDelayOrthogonal();
        moveDiagonalDelay = controller.getRobotType().moveDelayDiagonal();
        robot = controller.getRobot();
        isInAir = controller.getRobotType() == RobotType.ARCHON || controller.getRobotType() == RobotType.SCOUT;
        requests = new ArrayList<EnergonTransferRequest>();
        lowAllyRequests = new ArrayList<LowAllyRequest>();
        oldEnemies = new ArrayList<Integer>();
        oldLocations = new ArrayList<MapLocation>();
        lowEnergonLevel = controller.getRobotType().maxEnergon() * .3;
        robotCache = new RobotCache();
    }

    public void p(String s) {
        if(false)
            return;

        if(controller.getRobot().getID() == 109)
            System.out.println(s);
    }
    
    public void run() throws Exception {
    	myTeam = controller.getTeam();
        while (true) {
            if(isEnergonLow())
                requestEnergonTransfer();
        }
    }
    
    /***************************************************************************
     * SENSING CODE
     **************************************************************************/

    /**
     * Returns a 2D array with MapLocations or null objects based on the actual tiles this robot can sense.
     */
    public MapLocation[][] getSensibleTiles() {
        // return int locations of all tiles i can sense
        int radius = controller.getRobotType().sensorRadius();
        int size = radius*2+1;
        MapLocation[][] tiles = new MapLocation[size][size];
        int currentX = controller.getLocation().getX();
        int currentY = controller.getLocation().getY();

        for(int x = -radius; x <= radius; x++) {
            for(int y = -radius; y <= radius; y++) {
                MapLocation location = new MapLocation(x+currentX, y+currentY);
                if(controller.canSenseSquare(location))
                    tiles[x+radius][y+radius] = location;
                else
                    tiles[x+radius][y+radius] = null;
            }
        }

        return tiles;
    }

    public boolean runSensorCallbacks(MapData data) {
        boolean ret = true;
        if(data != null) {
            ret = tileSensedCallback(data);
            if(data.airRobot != null || data.groundRobot != null)
                ret = enemyInSightCallback(data) && ret;
            if(data.deposit != null)
                ret = fluxDepositInSightCallback(data) && ret;
        }
        return ret;
    }
    /**
     * Returns true if the unit can sense an enemy at the given location
     */
    public boolean canSenseEnemy(MapLocation enemyLocation)
    {
    	ArrayList<MapLocation> locations = senseEnemyRobotLocations();
    	System.out.println("Can sense enemy!");
    	for (MapLocation l : locations)
    		if (l.getX() == enemyLocation.getX() && l.getX() == enemyLocation.getY())
    		{    			
    			return true;
    		}
    	return false;
    }

    /**
     * Iteratres through each of the tiles in sensor range of the
     */
    public void senseAllTiles() {
        senseTiles(getSensibleTiles());
    }

    /**
     * This is used to sense the new tiles for robots with a 360 degree sensor angle.
     * Those robots pass in 3 int arrays representing the distance from the current robots location
     * of each new cell the robot can sense.  These distances are based on when the robot moves south,
     * west, and southwest respectively.
     *
     * Each of these is translated into a map location based on the direction the robot is facing.
     * ie: if the robot is moving east instead of west, the deltas are negated.
     *
     * The callback tileSensedCallback is called for each of the tiles.
     */
    public void senseDeltas(int[] verticalDeltas, int[] horizontalDeltas, int[] diagonalDeltas) {
        Direction dir = controller.getDirection();
        int[] directionDelta = getDirectionDelta(dir);
        int currentX = controller.getLocation().getX(), currentY = controller.getLocation().getY();

        int xDelta, yDelta, x, y;
        int[] deltas;

        if(directionDelta[0] == 0) {
            xDelta = 1;
            yDelta = directionDelta[1];
            deltas = verticalDeltas;
        } else if(directionDelta[1] == 0) {
            xDelta = directionDelta[0];
            yDelta = 1;
            deltas = horizontalDeltas;
        } else {
            xDelta = directionDelta[0];
            yDelta = directionDelta[1];
            deltas = diagonalDeltas;
        }

        for(int c = 0; c < deltas.length; c += 2) {
            x = currentX + deltas[c] * xDelta;
            y = currentY + deltas[c+1] * yDelta;
            runSensorCallbacks(senseTile(new MapLocation(x, y)));
        }
    }

    public ArrayList<RobotInfo> senseEnemyRobotInfoInSensorRange() {
    	ArrayList<RobotInfo> ret = new ArrayList<RobotInfo>();
        ArrayList<RobotInfo> ground = robotCache.getGroundRobotInfo(), air = robotCache.getAirRobotInfo();
        
        for(RobotInfo robot : ground)
            if(!robot.team.equals(myTeam))
                ret.add(robot);

        for(RobotInfo robot : air)
            if(!robot.team.equals(myTeam))
                ret.add(robot);

    	return ret;
    }

    public ArrayList<MapLocation> senseEnemyRobotLocations() {
    	ArrayList<RobotInfo> enemyRobots = senseEnemyRobotInfoInSensorRange();
    	ArrayList<MapLocation> returnList;
    	returnList = new ArrayList<MapLocation>();
    	for (RobotInfo r: enemyRobots)
    		returnList.add(r.location);
    	return returnList;
    }

    /**
     * Default method to update the map each time the robot moves.
     */
    public void senseNewTiles() {
        senseAllTiles();
    }

    /**
     * Sense the 8 tiles around the robot.
     */
    public MapData[] senseSurroundingSquares() {
        return senseSurroundingSquares(controller.getLocation());
    }

    public MapData[] senseSurroundingSquares(MapLocation location) {
        MapData[] ret = new MapData[9];
        int x = location.getX(), y = location.getY();
        
        ret[0] = senseTile(new MapLocation(x-1, y-1));
        ret[4] = senseTile(new MapLocation(x, y-1));
        ret[1] = senseTile(new MapLocation(x+1, y-1));

        ret[5] = senseTile(new MapLocation(x-1, y));
        ret[8] = senseTile(new MapLocation(x, y));
        ret[6] = senseTile(new MapLocation(x+1, y));

        ret[2] = senseTile(new MapLocation(x-1, y+1));
        ret[7] = senseTile(new MapLocation(x, y+1));
        ret[3] = senseTile(new MapLocation(x+1, y+1));

        return ret;
    }

    /**
     * Sense the terrain and block height of a tile, and whether the tile has a robot or flux deposit on it.
     * If the location is offmap, a MapData object will still be returned, but it will not be saved in the
     * mapstore object.  Walls will be automatically updated to reflect the new locations.
     */
    public MapData senseTile(MapLocation location) {
        TerrainTile tile = null;
        
        if(!controller.canSenseSquare(location))
            return null;
        
        try {
            tile = controller.senseTerrainTile(location);
        } catch (Exception e) {
            System.out.println("----Caught exception in senseTile1 tile: "+location.toString()+" Exception: "+e.toString());
        }

        // if the tile is off map, we do not want to store it in the database, cuz it will cause problems
        if(tile == null || tile.getType() == TerrainTile.TerrainType.OFF_MAP) {
            MapData data = new MapData(location);
            data.tile = tile;
            updateWalls(data);
            return data;
        }

        //grab the tile from the map store or create it if it doesn't exist because this tile is on the map
        try {
            MapData data = map.getOrCreate(location.getX(), location.getY());

            if(data.lastUpdate >= Clock.getRoundNum())
                return data;
            
            boolean updateWalls = data.tile == null;
            int terrainHeight = tile.getHeight();
            int blockHeight = controller.senseNumBlocksAtLocation(location);

            data.tile = tile;
            if(terrainHeight != data.terrainHeight || blockHeight != data.blockHeight) {
                //the block has changed so update the heights and wall locations
                data.terrainHeight = terrainHeight;
                data.blockHeight = blockHeight;
                data.height = terrainHeight + blockHeight;
            }

            data.airRobot = controller.senseAirRobotAtLocation(location);
            data.groundRobot = controller.senseGroundRobotAtLocation(location);

            if(!data.isFluxDeposit) {
                data.deposit = controller.senseFluxDepositAtLocation(location);
                if(data.deposit != null)
                    data.isFluxDeposit = true;
            }

            data.airRobotInfo = null;
            data.groundRobotInfo = null;
            data.depositInfo = null;
            
            if(data.airRobot != null)
                data.airRobotInfo = controller.senseRobotInfo(data.airRobot);
            if(data.groundRobot != null)
                data.groundRobotInfo = controller.senseRobotInfo(data.groundRobot);
            if(data.isFluxDeposit)
                data.depositInfo = controller.senseFluxDepositInfo(data.deposit);
            
            if(updateWalls)
                updateWalls(data);

            data.lastUpdate = Clock.getRoundNum();
            return data;
        } catch (Exception e) {
            System.out.println("----Caught exception in senseTile2 tile: "+location.toString()+" Exception: "+e.toString());
        }
        return null;
    }

    /**
     * Calls senseTile on each of the given tiles that are not null and then calls the callback
     * function tileSensedCallback for each of the resulting MapData objects.
     */
    public void senseTiles(MapLocation[][] tiles) {
        for(MapLocation[] row : tiles)
            for(MapLocation tile : row)
                if(tile != null)
                    if(!runSensorCallbacks(senseTile(tile)))
                        return;
    }

    /**
     * sets the players current goal
     */
    public void setGoal(int goal)
    {
    	this.currentGoal = goal;
    	controller.setIndicatorString(1, Goal.toString(goal));
        p("Changin goal to: "+Goal.toString(goal));
    }

    /**
     * Updates the wall and wallBounds location with the new data.
     *
     * The walls represent the first location that is off map, not the last location this is on
     * the map.
     *
     * The bounds variables represent the last location that we have seen on the map.  This
     * preveents two walls from changing when only one is off map.  For instance, if the player
     * is at the bottom of the map and senses a tile below the map, without the bounds variables
     * it would change both the right and botto walls.  With the bounds variables, it recognizes
     * that there can't be a wall there because it already saw an on map tile further to the
     * right of it.
     */
    public void updateWalls(MapData data) {
        if(data.tile == null)
            return;

        if(data.tile.getType() == TerrainTile.TerrainType.OFF_MAP) {
            if(data.x > rightWallBounds && data.x < rightWall)
                rightWall = data.x;
            if(data.x < leftWallBounds && data.x > leftWall)
                leftWall = data.x;

            if(data.y < topWallBounds && data.y > topWall)
                topWall = data.y;
            if(data.y > bottomWallBounds && data.y < bottomWall)
                bottomWall = data.y;
        } else if(data.tile.getType() == TerrainTile.TerrainType.LAND) {
            if(data.x > rightWallBounds) {
                rightWallBounds = data.x;
                if(rightWall <= rightWallBounds)
                    rightWall = rightWallBounds + 1;
            }
            if(data.x < leftWallBounds) {
                leftWallBounds = data.x;
                if(leftWall >= leftWallBounds)
                    leftWall = leftWallBounds - 1;
            }

            if(data.y > bottomWallBounds) {
                bottomWallBounds = data.y;
                if(bottomWall <= bottomWallBounds)
                    bottomWall = bottomWallBounds + 1;
            }
            if(data.y < topWallBounds) {
                topWallBounds = data.y;
                if(topWall >= topWallBounds)
                    topWall = topWallBounds - 1;
            }
        }
    }

    class RobotCache {
        public int airSensed = Integer.MIN_VALUE, groundSensed = Integer.MIN_VALUE,
                airInfoSensed = Integer.MIN_VALUE, groundInfoSensed = Integer.MIN_VALUE;
        public Robot[] air, ground;
        public ArrayList<RobotInfo> airInfo, groundInfo;
        public int oldDataTolerance = 1;

        public RobotCache() {

        }

        public Robot[] getAirRobots() {
            if(airSensed >= Clock.getRoundNum()-oldDataTolerance) {
                return air;
            }

            try {
                air = controller.senseNearbyAirRobots();
                airSensed = Clock.getRoundNum();
            } catch (Exception e) {
                System.out.println("----Caught Exception in getAirRobots.  Exception: "+e.toString());
            }
            return air;
        }

        public Robot[] getGroundRobots() {
            if(groundSensed >= Clock.getRoundNum()-oldDataTolerance) {
                return ground;
            }

            try {
                ground = controller.senseNearbyGroundRobots();
                groundSensed = Clock.getRoundNum();
            } catch (Exception e) {
                System.out.println("----Caught Exception in getGroundRobots.  Exception: "+e.toString());
            }
            return ground;
        }

        public ArrayList<RobotInfo> getAirRobotInfo() {
            if(airInfoSensed >= Clock.getRoundNum()-oldDataTolerance) {
                return airInfo;
            }

            getAirRobots();
            airInfo = new ArrayList<RobotInfo>();
            for(Robot robot : air) {
                try {
                    if(controller.canSenseObject(robot))
                        airInfo.add(controller.senseRobotInfo(robot));
                } catch (Exception e) {
                    System.out.println("----Caught Exception in getAirRobotInfo.  Exception: "+e.toString());
                }
            }
            airInfoSensed = Clock.getRoundNum();
            return airInfo;
        }

        public ArrayList<RobotInfo> getGroundRobotInfo() {
            if(groundInfoSensed >= Clock.getRoundNum()-oldDataTolerance) {
                return groundInfo;
            }

            getGroundRobots();
            groundInfo = new ArrayList<RobotInfo>();
            for(Robot robot : ground) {
                try {
                    if(controller.canSenseObject(robot))
                        groundInfo.add(controller.senseRobotInfo(robot));
                } catch (Exception e) {
                    System.out.println("----Caught Exception in getGroundRobotInfo.  Exception: "+e.toString());
                }
            }
            groundInfoSensed = Clock.getRoundNum();
            return groundInfo;
        }
    }

    /**************************************************************************
     *  ENERGON CODE
     **************************************************************************/

    /**
     * Auto energon transfers
     */
    public void autoTransferEnergon() {
        //p("auto transfer energon");
        MapData[] data = senseSurroundingSquares();
        for(MapData location : data) {
            if(location != null) {
                //p(location.toStringFull());
                if(location.airRobot != null && location.airRobotInfo.type != RobotType.ARCHON && location.airRobotInfo.team == myTeam) {
                    int amount = calculateEnergonRequestAmount(location.airRobotInfo);
                    if(amount >= 1) {
                        requests.add(new EnergonTransferRequest(location.toMapLocation(), true, amount));
                        //p("adding request: "+requests.get(requests.size()-1).toString());
                    }
                } if (location.groundRobot != null && location.groundRobotInfo.team == myTeam) {
                    int amount = calculateEnergonRequestAmount(location.groundRobotInfo);
                    if(amount >= 1) {
                        requests.add(new EnergonTransferRequest(location.toMapLocation(), false, amount));
                        //p("adding request: "+requests.get(requests.size()-1).toString());
                    }
                }
            }
        }
    }
    /**
     * Auto energon transfers between units
     */
    public void autoTransferEnergonBetweenUnits() {
        if(controller.getEnergonLevel() < controller.getRobotType().maxEnergon() / 2)
            return;
        if(!(controller.getRobotType() == RobotType.CANNON || controller.getRobotType() == RobotType.CHANNELER)) {
            MapData squares[] = senseSurroundingSquares();
            RobotInfo min = null, cur = null;
            for(MapData square : squares) {
                if(square == null)
                    return;
                if(square.groundRobot != null) {
                    cur = square.groundRobotInfo;
                    if(min == null)
                        min = cur;
                    else {
                        double percent = (cur.energonLevel + cur.energonReserve)/cur.type.maxEnergon();
                        double minpercent = (min.energonLevel + min.energonReserve)/min.type.maxEnergon();
                        if(percent < minpercent)
                            min = cur;
                    }
                }
            }
            if(min == null)
                return;
            double amount = calculateEnergonRequestAmount(min);
            if (amount < 2)
                return;
            transferEnergon(amount, min.location, min.type.isAirborne());
        } else {
            if(lowAllyRequestsTurn+1 <= Clock.getRoundNum())
                return;
            LowAllyRequest min = null;
            double percent, minpercent=500;
            for(LowAllyRequest cur : lowAllyRequests) {
                if(!cur.location.isAdjacentTo(controller.getLocation()))
                    continue;
                if(min == null) {
                    min = cur;
                    minpercent = (min.level + min.reserve)/min.max;
                } else {
                    percent = (cur.level + cur.reserve)/cur.max;
                    if(percent < minpercent) {
                        min = cur;
                        minpercent = percent;
                    }
                }
            }

            if(min == null)
                return;

            double amount = calculateEnergonRequestAmount(min.level, min.reserve, min.max);
            if(amount < 2)
                return;
            amount = amount / 2;
            
            transferEnergon(amount, min.location, false);
        }
    }
    
    /**
     * Calculates the amount of energon needed to fill this robot so that neither the
     * energon reserve nor energon level overflows.  Returns -1 if less than 1 energon
     * is needed.
     */
    public int calculateEnergonRequestAmount(double currentLevel, double currentReserve, double maxLevel) {
        double maxReserve = GameConstants.ENERGON_RESERVE_SIZE;
        double eventualLevel = currentLevel + currentReserve;

        if(currentReserve >= maxReserve-1)
            return -1;

        if(eventualLevel >= maxLevel-1)
            return -1;

        double transferAmount = maxReserve - currentReserve;

        eventualLevel += transferAmount;
        if(eventualLevel >= maxLevel)
            transferAmount -= (eventualLevel - maxLevel);

        int requestAmount = (int)Math.round(transferAmount);
        if(requestAmount <= 1)
            return -1;

        return requestAmount;
    }

    public int calculateEnergonRequestAmount() {
        return calculateEnergonRequestAmount(controller.getEnergonLevel(), controller.getEnergonReserve(), controller.getMaxEnergonLevel());
    }

    public int calculateEnergonRequestAmount(RobotInfo info) {
        return calculateEnergonRequestAmount(info.energonLevel, info.energonReserve, info.maxEnergon);
    }

    /**
     * Returns true if the energon level is 90% of max
     */
    public boolean isEnergonFull() {
        return controller.getEnergonLevel()+controller.getEnergonReserve() > controller.getRobotType().maxEnergon()*.9;
    }

    /**
     * Returns true if the energon level plus energon reserve is less than the starting energon level
     */
    public boolean isEnergonLow() {
        double currentLevel = controller.getEnergonLevel(), currentReserve = controller.getEnergonReserve();
        return (currentReserve == 0 && currentLevel < lowEnergonLevel);
    }

    public boolean isEnergonLow(RobotInfo info) {
        double currentLevel = info.energonLevel, currentReserve = info.energonReserve;
        return (currentReserve == 0 && currentLevel < info.maxEnergon*.3);
    }

    /**
     * Processes all of the energon requests in the requests ArrayList.
     * If multiple requests were asked for, the method will only give each robot a percentage
     * of the requested energon so the archon is not killed.
     */
    public void processEnergonTransferRequests() {
        //p("processing requests");
        if(requests.size() > 0) {
            //p("multiple requests");
            double sum = 0;
            for(EnergonTransferRequest request : requests)
                sum += request.amount;

            double percent = 1;
            double amount = controller.getEnergonLevel();
            if(amount-5 < sum)
                percent = (sum) / amount-5;

            //p("sum: "+sum+" percent: "+percent+" amount: "+amount);
            for(EnergonTransferRequest request : requests) {
                //(request.toString());
                int result = transferEnergon(request.amount*percent, request.location, request.isAirUnit);
            }

            requests.clear();
        }
    }
    
    /**
     * Performs all the steps to request an energon transfer.
     * First, it calculates an amount of energon to request, enough so that the neither
     * the reserve not level overflows.
     *
     * The robot then attempts to move adjacent to the closest archon.  However, if the
     * archon moves while the robot is moving, the robot will try 2 more times to get adjacent
     * to the robot.
     */
    public int requestEnergonTransfer() {
        int amount = calculateEnergonRequestAmount();
        if(amount == -1)
            return Status.success;

        int tries = 3;
        MapLocation closest = findNearestArchon();
        while(closest != null && !closest.isAdjacentTo(controller.getLocation())) {
            int result = moveOnceTowardsLocation(closest);
        }

        if(closest == null || !closest.isAdjacentTo(controller.getLocation()))
            return Status.fail;

        sendLowEnergon(closest, calculateEnergonRequestAmount());
        controller.yield();

        return Status.success;
    }

    /**
     * Transfers the specified amount of energon to the robot as long as the robot is adjacent to the
     * archon, and the amount of energon requested will not reduce the archon's energon level to
     * just 1 energon.
     */
    public int transferEnergon(double amount, MapLocation location, boolean isAirUnit) {
        if(controller.getEnergonLevel()-1 < amount)
            return Status.notEnoughEnergon;

        if(amount < 0)
            return Status.success;

        //System.out.println("in transfer");
        /*if(!location.isAdjacentTo(controller.getLocation()))
            return Status.fail;*/
        
        RobotLevel level = isAirUnit ? RobotLevel.IN_AIR : RobotLevel.ON_GROUND;
        try {
            if(controller.canSenseSquare(location) && ((isAirUnit && controller.senseAirRobotAtLocation(location) == null) ||
                    (!isAirUnit && controller.senseGroundRobotAtLocation(location) == null)))
                return Status.fail;
            controller.transferEnergon(amount, location, level);
        } catch (Exception e) {
            //System.out.println("----Caught Exception in transferEnergon. amount: "+amount+
            //        " location: "+location.toString()+" isAirUnit: "+isAirUnit+" level: "+
            //        level.toString()+" Exception: "+e.toString());
            return Status.fail;
        }
        return Status.success;
    }

    /**
     * Represents a pending energon request.
     */
    class EnergonTransferRequest {
        public MapLocation location, archonLocation;
        public boolean isAirUnit;
        public double amount;

        public EnergonTransferRequest(MapLocation location, boolean isAirUnit, int amount) {
            this.location = location;
            this.isAirUnit = isAirUnit;
            this.amount = amount;
        }

        public EnergonTransferRequest(int amount, boolean isAirUnit) {
            this.isAirUnit = isAirUnit;
            this.amount = amount;
        }

        public String toString() {
            return "Location: "+location.toString()+" isAirUnit: "+isAirUnit+" Amount: "+amount;
        }
    }
    
    class LowAllyRequest {
        public MapLocation location;
        public int level, reserve, max;

        public LowAllyRequest(MapLocation location, int level, int reserve, int max) {
            this.location = location;
            this.level = level;
            this.reserve = reserve;
            this.max = max;
        }
    }

    
    /***************************************************************************
     * NAVIGATION CODE
     **************************************************************************/

    /**
     * Returns the number of turns to move between two tiles of the corresponding height.
     * For ground units, this takes into consideration the difference in tile heights.
     * For air units, this is overriden in their classes to only return the base movement delay.
     */
    public int calculateMovementDelay(int heightFrom, int heightTo, boolean diagonal) {
        int cost = diagonal ? moveDiagonalDelay : moveStraightDelay;
        int delta = heightFrom - heightTo;
        if(Math.abs(delta) <= 1)
            return cost;
        if(delta > 0)
            return cost + GameConstants.FALLING_PENALTY_RATE * delta;
        else
            return cost + GameConstants.CLIMBING_PENALTY_RATE * delta * delta;
    }

    /**
     * Returns true if the map coordinate is out of bounds of the map.  Map boundaries
     * are initialized to the min/max integer values so that they this will work even
     * if the boundaries haven't been discovered yet.
     */
    public boolean checkWalls(MapData square) {
        return checkWalls(square.toMapLocation());
    }

    public boolean checkWalls(MapLocation square) {
        return square.getX() <= leftWall || square.getX() >= rightWall || square.getY() <= topWall || square.getY() >= bottomWall;
    }

    /**
     * Causes the robot to the face the specified direction if necessary.
     */
    public int faceDirection(Direction dir) {
        if(dir == null)
            return Status.fail;
        
        if(controller.getDirection().equals(dir))
            return Status.success;

        if(dir.equals(Direction.OMNI))
            return Status.success;

        while(controller.hasActionSet() || controller.getRoundsUntilMovementIdle() != 0)
            controller.yield();

        try {
            controller.setDirection(dir);
            controller.yield();
            return Status.success;
        } catch (Exception e) {
            System.out.println("----Caught Exception in faceDirection with dir: "+dir.toString()+" Exception: "+e.toString());
        }

        return Status.fail;
    }

    /**
     * Calculates the direction needed to turn and face the given location, such as
     * for when an Archon needs to turn to face an empty space to spawn a unit.
     * The robot will then wait until it's movement is idle and proceed to turn to that direction.
     */
    public int faceLocation(MapLocation location) {
        Direction newDir = getDirection(new MapData(controller.getLocation()), new MapData(location));
        if (newDir == null)
        	return Status.fail;
        if(newDir == controller.getDirection())
            return Status.success;

        if(controller.hasActionSet())
            controller.yield();

        while(controller.getRoundsUntilMovementIdle() != 0)
            controller.yield();
        
        try {
            controller.setDirection(newDir);
            controller.yield();
        } catch (Exception e) {
            System.out.println("----Caught exception in faceLocation with MapLocation: "+location.toString()+" Exception: "+e.toString());
            return Status.fail;
        }
        return Status.success;
    }

    /**
     * Returns the MapLocation of the archon closest to this robot.
     */
    public MapLocation findNearestArchon() {
        MapLocation current = controller.getLocation();
        MapLocation[] locations = controller.senseAlliedArchons();

        MapLocation min = null;
        int minDistance = Integer.MAX_VALUE;

        for(MapLocation location : locations) {
            int distance = current.distanceSquaredTo(location);
            if(distance < minDistance && distance >= 1) {
                minDistance = distance;
                min = location;
            }
        }

        return min;
    }
    
    /**
     * Finds a tile path between two locations.
     * If the algorithm cannot find a path of 100 or fewer tiles, it returns null.
     *
     * The resulting path is a LinkedList of map locations.  There is no guarantee
     * that the tiles are on the map if the map borders have not been discovered by
     * this robot yet.  There is no guarantee that a path tile will not be obstructed
     * when the robot gets there.
     *
     * The algorithm performs astar search on the map tiles.  The cost function is the
     * number of turns to get from the start location to the current tile.  The heuristic
     * is the straight line distance number of turns to get to the goal.
     *
     * Rather than considering every possible square a robot could move to, the algorithm
     * only considers 3 new squares each time based on the previous direction of the robot.
     * For instance, if the robot moved up, it will only consider the 3 squares at the top,
     * there is no need to consider if the robot should move backwards.
     */
    public LinkedList<MapData> findPath(MapData start, MapData end) {
        //p(start.toString()+"  "+end.toString());
        LinkedList<PathLocation> states = new LinkedList<PathLocation>();
        ArrayList<PathLocation> newStates = new ArrayList<PathLocation>();
        Hashtable<String, Integer> seenStates = new Hashtable<String, Integer>();
        states.add(new PathLocation(start, null, end));
        PathLocation current = null;

        for(int c = 0; c < 100; c++) {
            newStates.clear();
            //for the current level, process each of the states by calculating the cost and estimate for each of the 8 surrounding squares
            //p("New Iteration: "+states.size());
            //p("States: "+states.toString());
            while(!states.isEmpty()) {
                current = states.removeFirst();
                //p("  Current: "+current.toString3());
                MapData[] squares = map.getForwardSquares(current.location.x, current.location.y, current.xDelta, current.yDelta, current.diagonal);
                //p("  Squares: "+squares[0].toString()+" "+squares[1].toString()+" "+squares[2].toString());
                for(MapData square : squares) {
                    // ensure this step is not out of bounds
                    if(checkWalls(square))
                        continue;

                    MapLocation squarel = square.toMapLocation();
                    try {
                        if(controller.canSenseSquare(squarel)) {
                            if(isInAir) {
                                if(controller.senseAirRobotAtLocation(squarel) != null)
                                    continue;
                            } else {
                                if(controller.senseGroundRobotAtLocation(squarel) != null)
                                    continue;
                            }
                        }
                    } catch (Exception e) {
                        System.out.println("----Caught exception in findPath while sense the square. Square: "+
                                squarel.toString()+" Exception: "+e.toString());
                    }

                    // check for the goal state
                    if(square.equals(end)) {
                        end.pathCost = current.cost + calculateMovementDelay(current.height, map.getHeight(end), true);
                        LinkedList<MapData> ret = new LinkedList<MapData>();
                        ret.addFirst(end);
                        // reverse the path pointed to by current.previous and remove the start location
                        while(current.previous != null) {
                            ret.addFirst(current.location);
                            current = current.previous;
                        }
                        return ret;
                    }

                    PathLocation state = new PathLocation(square, current, end);
                    //p("    New state: "+state.toString2());

                    // check if state has already been marked for consideration.  if it has been, but this is cheaper, then still consider it
                    String s = state.toString();
                    Integer prevCost = seenStates.get(s);
                    if(prevCost != null)
                        if(prevCost.compareTo(state.intCost) <= 0) {
                            //p("        rejecting square");
                            continue;
                        } else {
                            seenStates.remove(s);
                            newStates.remove(state);
                        }
                    seenStates.put(s, state.intCost);       
                    newStates.add(state);
                }
            }

            //newStates contains up to 32 different squares that we haven't yet considered.  choose the 3 cheapest to continue
            //p(newStates.size()+"  "+newStates.toString());
            for(int count = 0; count < 3 && count < newStates.size(); count++) {
                int minId = count;
                PathLocation min = newStates.get(count);
                for(int d = count+1; d < newStates.size(); d++) {
                    PathLocation test = newStates.get(d);
                    if(test.total < min.total) {
                        minId = d;
                        min = test;
                    }
                }
                newStates.set(minId, newStates.get(count));
                states.add(min);
            }
        }
        //p("returning null");
        //uhoh, we shouldn't be here WUT ARE WE GONNA DO? (return null)
        return null;
    }

    /**
     * Returns the direction object needed to move a robot from the start square to the end square.
     */
    public Direction getDirection(MapData start, MapData end) {
        int x = end.x - start.x;
        int y = end.y - start.y;
        return getDirection(x, y);
    }

    public Direction getDirection(MapLocation start, MapLocation end) {
        int x = end.getX() - start.getX();
        int y = end.getY() - start.getY();
        return getDirection(x, y);
    }

    public Direction getDirection(int x, int y) {
        if(y < 0) {
            if(x > 0)
                return Direction.NORTH_EAST;
            if(x == 0)
                return Direction.NORTH;
            if(x < 0)
                return Direction.NORTH_WEST;
        } else if (y == 0) {
            if(x > 0)
                return Direction.EAST;
            if(x < 0)
                return Direction.WEST;
        } else if (y > 0) {
            if(x > 0)
                return Direction.SOUTH_EAST;
            if(x == 0)
                return Direction.SOUTH;
            if(x < 0)
                return Direction.SOUTH_WEST;
        }
        return null;
    }

    /**
     * Returns the change to the location of an object if it moves one tile in the specified direction.
     */
    public int[] getDirectionDelta(Direction direction) {
        if(direction == Direction.NORTH_WEST)
            return new int[] {-1, -1};
        if(direction == Direction.NORTH)
            return new int[] {0, -1};
        if(direction == Direction.NORTH_EAST)
            return new int[] {1, -1};

        if(direction == Direction.EAST)
            return new int[] {1, 0};
        if(direction == Direction.WEST)
            return new int[] {-1, 0};

        if(direction == Direction.SOUTH_WEST)
            return new int[] {-1, 1};
        if(direction == Direction.SOUTH)
            return new int[] {0, 1};
        if(direction == Direction.SOUTH_EAST)
            return new int[] {1, 1};

        return new int[] {0, 0};
    }

    /**
     * Returns the Manhattan Distance
     */
    public int getDistanceTo(MapLocation location) {
        int x = location.getX() - controller.getLocation().getX();
        int y = location.getY() - controller.getLocation().getY();
        return Math.abs(x) + Math.abs(y);
    }

    /**
     * Returns the Manhattan Distance to the nearest archon
     */
    public int getDistanceToNearestArchon() {
        MapLocation location = findNearestArchon();
        int x = location.getX() - controller.getLocation().getX();
        int y = location.getY() - controller.getLocation().getY();
        return Math.abs(x) + Math.abs(y);
    }

    /**
     * Returns the first direction that the robot can move in, starting with the given direction.
     */
    public Direction getMoveableDirection(Direction dir) {
        if(dir == null)
            return null;
        Direction leftDir = dir, rightDir = dir;
        if(controller.canMove(dir))
            return dir;
        else {
            for(int d = 0; d < 3; d++) {
                leftDir = leftDir.rotateLeft();
                rightDir = rightDir.rotateRight();

                if(controller.canMove(leftDir))
                    return leftDir;
                if(controller.canMove(rightDir))
                    return rightDir;
            }
        }
        return null;
    }

    public Direction getMoveableWorkerDirection(Direction dir) {
        p("in get moveable worker direction");
        if(dir == null)
            return null;
        boolean messageSent = false;
        int pauseTurns = 5;
        do {
            if(controller.canMove(dir))
                return dir;
            if(controller.canMove(dir.rotateLeft()))
                return dir.rotateLeft();
            if(controller.canMove(dir.rotateRight()))
                return dir.rotateRight();
            if(controller.canMove(dir.rotateRight().rotateRight()))
                return dir.rotateRight().rotateRight();
            if(controller.canMove(dir.rotateLeft().rotateLeft()))
                return dir.rotateLeft().rotateLeft();

            if(!messageSent) {
                p("sending message");
                messageSent = true;
                sendMove(controller.getLocation().add(dir));
            }
            pauseTurns--;
            controller.yield();
        } while(pauseTurns >= 0);

        p("returning getMoveableDirection");
        return getMoveableDirection(dir);
    }

    /**
     * Returns an array of the 8 map locations around a robot.  These are sorted so
     * that the first location is the one the robot is facing, and then the 2 next to
     * that location, the 2 next to that, and so on.  The last location is the tile directly
     * behind the robot.
     */
    public MapData[] getOrderedMapLocations() {
        Direction cur = controller.getDirection(), left, right;
        MapLocation start = controller.getLocation();


        MapData[] ret = new MapData[8];
        ret[0] = map.getNotNull(start.add(cur));
        ret[7] = map.getNotNull(start.subtract(cur));

        for(int c = 1; c < 7; c++) {
            left = cur.rotateLeft();
            right = cur.rotateRight();
            ret[c] = map.getNotNull(start.add(right));
            c++;
            ret[c] = map.getNotNull(start.add(left));
        }

        return ret;
    }

    public void checkBlockedUnitsAndWait(MapLocation location) {
        boolean messageSent = false;
        int pauseCount = 5;
        do {
            try {
                if(controller.canSenseSquare(location) && controller.senseGroundRobotAtLocation(location) != null) {
                    if(!messageSent) {
                        sendMove(location);
                        messageSent = true;
                    }
                    controller.yield();
                } else {
                    break;
                }
            } catch (Exception e) {

            }
            pauseCount--;
        } while(pauseCount >= 0);
    }

    public int goByBugging(MapData end) {
        MapData previous = null;
        Direction previousDir = null, dir = controller.getDirection();
        int count = 0;
        int oppositeCount = 0;
        controller.setIndicatorString(2, "Goal: "+end.toMapLocation().toString());
        for(int c = 0; c < 100; c++) {
            controller.setIndicatorString(0, "Loc: "+controller.getLocation().toString());
            previous = map.getNotNull(controller.getLocation());
            if(controller.getLocation().equals(end.toMapLocation()))
                return Status.success;

            yieldMoving();
            faceLocation(end.toMapLocation());

            if(controller.getLocation().isAdjacentTo(end.toMapLocation())) {
                if(!controller.canMove(getDirection(controller.getLocation(), end.toMapLocation()))) {
                    try {
                        if(controller.canSenseSquare(end.toMapLocation()) && (controller.senseGroundRobotAtLocation(end.toMapLocation()) != null)) {
                            checkBlockedUnitsAndWait(end.toMapLocation());
                        } else {
                            // no one is blocking us but we can't go there, let's fail
                            return Status.fail;
                        }
                    } catch (Exception e) {}
                }
            }

            previous = map.getNotNull(controller.getLocation());
            if(controller.getRobotType().equals(RobotType.WORKER))
                dir = getMoveableWorkerDirection(getDirection(previous, end));
            else
                dir = getMoveableDirection(getDirection(previous, end));

            if(dir == null) {
                System.out.println("null direction");
                controller.yield();
                continue;
            }

            Direction opposite = dir.opposite();
            if(opposite.equals(previousDir) || opposite.rotateLeft().equals(previousDir) || opposite.rotateRight().equals(previousDir)) {
                p("not gonna go that way");
                oppositeCount++;
                if(oppositeCount > 5)
                    return Status.fail;
                controller.yield();
                continue;
            }
            oppositeCount = 0;

            if(faceDirection(dir) != Status.success) {
                p("couldnt face that way");
                controller.yield();
                continue;
            }
            
            previousDir = dir;

            yieldMoving();

            try {
                boolean good = false;
                for(int d = 0; d < 2; d++) {
                    if(controller.canMove(dir)) {
                        if(!beforeMovementCallback(map.getNotNull(controller.getLocation().add(dir))))
                            return Status.success;
                        controller.moveForward();
                        controller.yield();
                        if(!pathStepTakenCallback())
                            return Status.success;
                        good = true;
                        count = 0;
                        break;
                    }
                    p("yielding");
                    controller.yield();
                }
                if(!good) {
                    count++;
                    if(count >= 3)
                        return Status.cantMoveThere;
                }

            } catch (Exception e) {
                System.out.println("----Caught Exception in go dir: "+dir.toString()+" Exception: "+e.toString());
            }
        }
        return Status.fail;
    }
    /**
     * Makes the robot move from its current location to end.
     * The method first calculates a path to the location.  It then calls the pathCalculateCallback, 
     * which can be overriden in the base class.  If the callback returns false, the method halts.
     *
     * The method then proceeds to traverse the entire path.  If it encounters a simple obstacle,
     * such as one unit in the way, it will attempt to go around.  More complex obstacles will
     * cause it to completely retry the path.
     *
     * Each time the robot moves, it calls the pathStepTakenCallback, which can be 
     * overriden in the base class.  If the callback returns false, the method returns.
     * 
     * TODO: validate we are at the goal
     * TODO: repair path
     */
    public int go(MapData end) {
        return go(end, 0);
    }

    public int go(MapData end, int depth) {
        if(depth > 5)
            return Status.fail;
        
        MapData previous = map.getNotNull(controller.getLocation());
        //p("get path");
        LinkedList<MapData> path = findPath(previous, end);
        //p("got path");
        if(path == null)
            return Status.fail;
        
        if(!pathCalculatedCallback(path))
            return Status.success;
        
        //System.out.println(controller.getRobot().getID()+": "+previous.toString()+"   "+end.toString()+"   "+path.toString());
        Direction dir = controller.getDirection();
        while(!path.isEmpty()) {
            yieldMoving();

            MapData step = path.removeFirst();
            if(controller.canSenseSquare(step.toMapLocation())) {
                MapData updatedStep = senseTile(step.toMapLocation());
                step = updatedStep;

                if((isInAir && step.airRobot != null) || (!isInAir && step.groundRobot != null)) {
                    // the path is blocked
                    if(path.isEmpty()) {
                        // the goal is blocked
                        return Status.goalBlocked;
                    }
                    return go(end, depth+1);
                }
            }
            //find if we need to change direction
            Direction newDirection = getDirection(previous, step);
            faceDirection(newDirection);

            //check if the square is free
            if(!controller.canMove(dir)) {
                //uhoh, we can't move here, prolly cuz a robot is in the way, time to plot a path around?
                //check if the square right next to the obstacle is free
                //just plot a path around and recursively call go?
                return go(end, depth+1);
            } else {
                try {
                    if(!beforeMovementCallback(step))
                        return Status.success;

                    if(controller.hasActionSet())
                        controller.yield();

                    controller.moveForward();
                    controller.yield();
                } catch (Exception e) {
                    System.out.println("----Caught exception in go while moving forward. Exception: "+e.toString());
                    return Status.fail;
                }
                if(!pathStepTakenCallback())
                    return Status.success;

            }
        }

        return Status.success;
    }

    /**
     * Returns true if there is no Air or Ground unit at the given location.
     * If a robot is blind (channeler), this method should not be called.  It does
     * not check if the robot can sense at that location.
     */
    public boolean isLocationFree(MapLocation location, boolean isAirUnit) {
        try {
            if(checkWalls(location))
                return false;
            
            if(!controller.senseTerrainTile(location).isTraversableAtHeight((isAirUnit ? RobotLevel.IN_AIR : RobotLevel.ON_GROUND)))
                return false;

            if(isAirUnit)
                return controller.senseAirRobotAtLocation(location) == null;
            else
                return controller.senseGroundRobotAtLocation(location) == null;
        } catch (Exception e) {
            System.out.println("----Caught Exception in isLocationFree location: "+location.toString()+" isAirUnit: "+
                    isAirUnit+" Exception: "+e.toString());
            return false;
        }
    }

    /**
     * Moves the robot one step forward if possible.
     */
    public int moveOnce(Direction dir) {
        if(faceDirection(dir) != Status.success)
            return Status.fail;

        yieldMoving();

        try {
            for(int c = 0; c < 2; c++) {
                if(controller.canMove(dir)) {
                    beforeMovementCallback(map.get(controller.getLocation().add(dir)));
                    controller.moveForward();
                    controller.yield();
                    pathStepTakenCallback();
                    return Status.success;
                }
                controller.yield();
            }
            return Status.cantMoveThere;

        } catch (Exception e) {
            System.out.println("----Caught Exception in moveOnce dir: "+dir.toString()+" Exception: "+e.toString());
        }
        return Status.fail;
    }

    public int moveOnceTowardsLocation(MapLocation location) {
        Direction dir = getDirection(controller.getLocation(), location);
        dir = getMoveableDirection(dir);
        
        if(dir == null)
            return Status.fail;
        if(!directionCalculatedCallback(dir))
            return Status.success;

        return moveOnce(dir);
    }

    public void yieldMoving() {
        String cur = Goal.toString(currentGoal);
        controller.setIndicatorString(1, "yielding");
        while (controller.hasActionSet() || controller.getRoundsUntilMovementIdle() != 0)
            controller.yield();
        controller.setIndicatorString(1, cur);
    }

    class PathLocation {
        public MapData location;
        public PathLocation previous;
        public int cost, estimate, height, total, xDelta, yDelta;
        public boolean diagonal;
        public Integer intCost;

        public PathLocation(MapData location, PathLocation previous, MapData goal) {
            this.location = location;
            this.previous = previous;
            height = map.getHeight(location);
            if(previous != null) {
                //non diagonal squares will have an x or y coordinate which is the same in both the current location and the next one
                diagonal = !(location.x == previous.location.x || location.y == previous.location.y);
                xDelta = location.x - previous.location.x;
                yDelta = location.y - previous.location.y;
                cost = calculateMovementDelay(height, previous.height, diagonal) + previous.cost;
                //if we have to change direction
                if(xDelta != previous.xDelta || yDelta != previous.yDelta)
                    cost++;
            } else {
                // the first state has no previous data
                diagonal = false;
                cost = 0;
                xDelta = -2;
                yDelta = -2;
            }
            estimate = calculateEstimate(goal);
            total = cost + estimate;
            intCost = new Integer(total);
        }

        /**
         * Heuristic function for astar search.
         *
         * Currently, this calculates the straight line distance to the goal and returns the number of turns to traverse that number of squares.
         * TODO: Consider the difference in height between the current location and the goal.
         */
        public int calculateEstimate(MapData goal) {
            int x = Math.abs(goal.x - location.x), y = Math.abs(goal.y - location.y);
            int diagonalSquares = (int) Math.sqrt(x*x + y*y);
            return diagonalSquares * moveDiagonalDelay;
        }

        /**
         * Returns true if the two objects have the same location only.
         */
        public boolean equals(Object o) {
            PathLocation other = (PathLocation) o;
            return other.location.equals(location);
        }

        public String toString() {
            return location.x+" "+location.y;
        }
    }

    /***************************************************************************
     * CALLBACKS
     **************************************************************************/

    /**
     * Callback right before the moveForward method is called so a robot can check
     * if the tile is free to reevaluate its goals.
     */
    public boolean beforeMovementCallback(MapData location) {
        return true;
    }

    /**
     * Called when the direction in moveOnceTowardsLocation is calculated.
     */
    public boolean directionCalculatedCallback(Direction dir) {
        return true;
    }
    
    /**
     * Callback when an enemy is spotted in the sense methods.
     * The MapData object contains the robots.
     * Return false to return from the calling method.
     */
    public boolean enemyInSightCallback(MapData location) {
        return true;
    }

    /**
     * Callback for when a flux deposit is spotted.
     */
    public boolean fluxDepositInSightCallback(MapData location) {
        return true;
    }
    
    /**
     * Callback in the go method when a path is calculated.  The robot can override this to,
     * for exaple, make sure it has enough energon to get back to the archon.
     * If the callback returns false, the go method returns.
     */
    public boolean pathCalculatedCallback(LinkedList<MapData> path) {
        return true;
    }

    /**
     * Callback in the go method each time the robot takes a step.  One use would be to
     * check if an enemy is in sight, each time you move.
     * If the callback returns false, the go method returns.
     */
    public boolean pathStepTakenCallback() {
        senseNewTiles();
        return true;
    }

    /**
     * Callback for when a tile is sensed.  Return false to stop sensing.
     */
    public boolean tileSensedCallback(MapData tile) {
        return true;
    }

    /***************************************************************************
     * BROADCAST MESSAGING CODE
     **************************************************************************/

    //sendMessage (int[] ints, MapLocation[] locations, String[] strings)
    //sets a broadcast message to send at the end of turn.
    public boolean sendMessage() {
    	Message message = new Message();
    	message.ints = new int[messageInts.size()+3];
    	message.ints[0] = KEY1;
    	message.ints[1] = KEY2;
    	message.ints[2] = robot.getID();
        boolean good = true;
        
    	for (int i = 3; i < messageInts.size()+3; i++)
    		message.ints[i] = messageInts.get(i-3);
    	
    	message.locations = new MapLocation[messageLocations.size()];
    	for (int i=0;i<message.locations.length;i++)
    		message.locations[i] = messageLocations.get(i);

    	message.strings = new String[messageStrings.size()];
    	for (int i=0;i<message.strings.length;i++)
    		message.strings[i] = messageStrings.get(i);
        
    	try {
    		if (controller.hasBroadcastMessage())
    			controller.clearBroadcast();
    		
    		controller.broadcast(message);

    		good = true;
    	} catch (Exception e){
    		e.printStackTrace();
            p("----Caught Exception in sendMessage.  message: "+message.toString()+" Exception: "+e.toString());
            good = false;
        }
        return good;
    }
    
    public void parseMessages() {
        Message[] messages = controller.getAllMessages();
        for(Message message : messages)
            processMessage(message);
    }

    public void processMessage(Message message)
    {
    	if (message != null)
    	{
    		
    		//We got a message!
            /*String out = "";
            for(int c = 0; c < message.ints.length; c++)
                out += " ["+c+"]="+message.ints[c];
            p("Message Received: "+out);*/

    		//is it ours?
    		if (message.ints == null || message.ints.length < 3 || message.ints[0] != KEY1 || message.ints[1] != KEY2)
    			return;

    		int senderID = message.ints[2];

    		//Read it!
    		int locationIndex = 0;
    		int stringIndex = 0;
    		int messageID = -1, recipientID = -1;

    		for(int i = 3; i < message.ints.length; i+= getMessageLength(messageID)) {
                messageID = message.ints[i];
                //System.out.println(messageID);
                recipientID = message.ints[i+1];
                switch (messageID) {
                    case BroadcastMessage.UNDER_ATTACK:
                        ;//someone is under attack!!!!
                    break;
                    case BroadcastMessage.ENEMY_IN_SIGHT:
                        enemyInSight(message.locations[locationIndex+1], message.ints[i+2], message.strings[stringIndex]);//theres an enemy!
                        locationIndex+=2;
                        stringIndex++;
                        //this will broadcast the unit's location (locations[0]) and the enemy's location(locations[1])
                        //
                    break;
                    case BroadcastMessage.FLUX_IN_SIGHT:
                        ;//theres some flux!
                        //locations[0] is the unit's location, locations[1] is the flux's location
                    break;
                    case BroadcastMessage.MAP_INFO:
                        ;//update of mapinfo can be very many locations[]
                    break;
                    case BroadcastMessage.NEW_UNIT:
                        newUnit(senderID, message.locations[0], message.strings[0]);//theres a new unit! I should send my map info!
                    break;
                    case BroadcastMessage.SCOUT_ALLIED_UNIT_RELAY:
                        if(controller.getRobotType() == RobotType.SCOUT)
                            setGoal(Goal.alliedUnitRelay);
                    break;
                    case BroadcastMessage.LOW_ALLIED_UNITS:
                        int count = message.ints[i+2];
                        int index = i+3;
                        if(controller.getRobotType() != RobotType.ARCHON) {
                            lowAllyRequests.clear();
                            for(int c = 0; c < count; c++) {
                                MapLocation location = message.locations[locationIndex];
                                int level = message.ints[index];
                                int reserve = message.ints[index+1];
                                int max = message.ints[index+2];

                                lowAllyRequests.add(new LowAllyRequest(location, level, reserve, max));
                                
                                index += 3;
                                locationIndex++;
                            }
                            lowAllyRequestsTurn = Clock.getRoundNum();
                            i = index+1;
                        } else {
                            i = i+3+(count*3)+1;
                        }
                    break;
                    case BroadcastMessage.LOW_ENERGON:
                        //make sure the message is intended for me
                        if(message.locations[locationIndex+1].equals(controller.getLocation())) {
                            requests.add(new EnergonTransferRequest(message.locations[locationIndex], message.ints[i+3]==1, message.ints[i+2]));
                            locationIndex += 2;
                        }
                    break;
                    case BroadcastMessage.FIND_BLOCKS:
                    	locationIndex++;
                        if(message.ints[i+1] == controller.getRobot().getID()) {
                            Direction dir = Direction.NONE;
                            switch(message.ints[i+2])
                            {
                                case 0:
                                    dir = Direction.NORTH;
                                break;
                                case 1:
                                    dir = Direction.EAST;
                                break;
                                case 2:
                                    dir = Direction.SOUTH;
                                break;
                                case 3:
                                    dir = Direction.WEST;
                                break;
                            }
                            findBlocks(message.locations[locationIndex-1], dir);
                        }
                    break;
                    case BroadcastMessage.MOVE:
                    	locationIndex++;
                    	
                    	if (!controller.getRobotType().isAirborne() && controller.getLocation().equals(message.locations[locationIndex-1])) {
                            moveOnce(getMoveableDirection(Direction.NORTH));
                    	}
                    	
                    break;
                    case BroadcastMessage.PONG:
                        for (int j = 2; i < message.ints.length; i++)
                            switch(message.ints[i]) {
                                case BroadcastMessage.ENEMY_IN_SIGHT:
                                    ;//this unit has seen an enemy in the past 100 turns
                                break;
                                case BroadcastMessage.FLUX_IN_SIGHT:
                                    ;//this unit can see flux
                                break;
                                case BroadcastMessage.UNDER_ATTACK:
                                    ;//this unit has been recently attacked
                                break;
                            }
                        ;//locations[0] is the location of the unit
                        //locations[i] corresponds to the location for the status
                    break;
                    case BroadcastMessage.FOLLOW_REQUEST:
                    	
                    	if (controller.getRobotType()==RobotType.ARCHON)                    
                    		followRequest(message.ints[i+2], senderID);                    	
                    	else if (recipientID == robot.getID() && currentGoal != Goal.followingArchon)
                    	{                    		                    		
                    		setGoal(Goal.followingArchon);
                    		go(new MapData(message.locations[locationIndex]));
                    		followingArchonNumber = senderID;
                    		locationIndex++;
                    	}
                    break;
                    case BroadcastMessage.SUPPORT:
                    	
                    break;
                }
            }
    	}
    }
    public void followRequest(int archonNum, int id)
    {
    	;//to be overridden
    }
    public boolean broadcastMap(MapLocation[] locations)
    {
    	int[] ints = new int[2];
    	ints[1] = BroadcastMessage.MAP_INFO;
    	ints[0] = robot.getID();
    	return addMessage(ints, null, locations);
    }
    
    /**********************************************
     *  BROADCASTING A MESSAGE CODE
     *  ints[0] and ints[1] are the KEYS in the message
     *  ints[2] is ints[0] in the messages.
     *  ints[2] is senderID
     *  ints[3] is messageCode
     *  ints[4] is recepientID (-1 if its for everyone)
     *  
     **********************************************/
    public int getMessageLength(int messageID)
    {
    	switch(messageID)
    	{
    		case BroadcastMessage.ENEMY_IN_SIGHT:
    			return 3;
    		case BroadcastMessage.FLUX_IN_SIGHT:
    			return 2;
    		case BroadcastMessage.LOW_ENERGON:
    			return 4;
    		case BroadcastMessage.NEW_UNIT:
    			return 2;
    		case BroadcastMessage.FOLLOW_REQUEST:
    			return 3;
    		case BroadcastMessage.FIND_BLOCKS:
    			return 3;
    		case BroadcastMessage.MOVE:
    			return 2;
    		case BroadcastMessage.LOW_ALLIED_UNITS:
    			return 0;
    		case BroadcastMessage.SCOUT_ALLIED_UNIT_RELAY:
    			return 2;
    	}
    	return -1;
    }
    /**
     * is called when received an enemyInSight message - to be overloaded
     * 
     **/
    public void enemyInSight(MapLocation enemyLocation, int enemyID, String enemyType) {
    }
    /**
     *  is called when received a newUnit message - to be overloaded;
     */
    public void newUnit(int senderID, MapLocation location, String robotType){    	
    }
    /**
     *  is called when Find Blocks message is receive to be overloaded;
     */
    public void findBlocks(MapLocation fluxLocation, Direction d){
    	
    }
    /**
     * callback for an enemy in sight to be overridden
     * 
     * @param enemies
     */
    public void enemyInSight(ArrayList<RobotInfo> enemies)
    {
    	
    }
    /**
     * This method sends enemyInSight message broadcast for all enemies in sight
     */
    public void sendMessageForEnemyRobots()
    {
        ArrayList<RobotInfo> enemies = senseEnemyRobotInfoInSensorRange();
        //System.out.println(enemies.toString());
        //int id;
        //RobotInfo info;
        //MapLocation oldLocation;
        for (RobotInfo robot : enemies)
        {
        		int[] data = {(int)robot.energonLevel, -1};
        		String robotType = robot.type.toString();

                //System.out.println("Sending message: "+robot.toString()+" "+robot.location.toString());
        		sendEnemyInSight(robot.location, data, robotType);
        	/*id = robot.getID();
        	try
        	{
        		info = controller.senseRobotInfo(robot);
        	}
        	catch (GameActionException ex)
        	{
        		//p("------------------------------Cannot sense Robot Info in scout method");
        		info = null;
        		continue;
        	}
        	
        	if (!oldEnemies.contains(id))
        	{
        		oldEnemies.add(id);
        		oldLocations.add(info.location);
        		int[] data = {(int)info.energonLevel, id};
        		String robotType = info.type.toString();
        		sendEnemyInSight(info.location, data, robotType);
        	}
        	else
        	{
        		oldLocation = oldLocations.get(oldEnemies.indexOf(id));
        		if (oldLocation.getX() != info.location.getX() || oldLocation.getY() != info.location.getY())
        		{
        			int[] data = {(int)info.energonLevel, id};
            		String robotType = info.type.toString();
            		sendEnemyInSight(info.location, data, robotType);
                }
            }*/
        }
        if(enemies.size() > 0)
        {
            sendMessage();
            enemyInSight(enemies);
        }
	}
    
    public boolean addMessage(int[] ints, String[] strings, MapLocation[] locations)
    {
        if(!controller.hasBroadcastMessage())
            clearMessages();
        
        int i = 0;
        if (ints != null)
            for (i = 0; i < ints.length; i++)
                messageInts.add(ints[i]);
        if (strings != null)
            for (i = 0; strings!=null && i < strings.length; i++)
                messageStrings.add(strings[i]);
        if (locations != null)
            for (i = 0; i < locations.length; i++)
                messageLocations.add(locations[i]);

        sendMessage();
        return true;
    }

    /*
     * send message methods
     */
    public void clearMessages() {
        messageInts.clear();
        messageStrings.clear();
        messageLocations.clear();
    }

    public boolean sendScoutAlliedUnitRelay() {
        return addMessage(new int[] {BroadcastMessage.SCOUT_ALLIED_UNIT_RELAY, -1}, null, null);
    }
    public boolean sendFluxInSight(MapLocation location)
    {
    	int[] ints = new int[2];
    	ints[0] = BroadcastMessage.FLUX_IN_SIGHT;
    	MapLocation[] locations = new MapLocation[2];
    	locations[0] = controller.getLocation();
    	locations[1] = location;
    	String[] strings = null;
    	return addMessage(ints, strings, locations);
    }

    public boolean sendMove(MapLocation location)
    {
    	int[] ints = new int[2];
    	ints[0] = BroadcastMessage.MOVE;
    	ints[1] = -1;
    	MapLocation[] locations = new MapLocation[1];
    	locations[0] = location;
    	String[] strings = null;
    	p("Sent Move");
    	return addMessage(ints, strings, locations);
    }
    public boolean sendLowEnergon(MapLocation archonLocation, int amount)
    {
    	int[] ints = new int[4];
    	ints[0] = BroadcastMessage.LOW_ENERGON;
        ints[1] = -1;
        ints[2] = amount;
        ints[3] = controller.getRobotType().isAirborne() ? 1 : 0;
        
    	MapLocation[] locations = new MapLocation[2];
    	locations[0] = controller.getLocation();
    	locations[1] = archonLocation;
        
    	String[] strings = null;

    	return addMessage(ints, strings, locations);
    }
    
    public boolean sendEnemyInSight(MapLocation location, int[] data, String robotType)
    {
    	int[] ints = new int[3];
    	ints[0] = BroadcastMessage.ENEMY_IN_SIGHT;
    	ints[1] = -1;
    	//the type of enemy
    	ints[2] = data[0];
    	MapLocation[] locations = new MapLocation[2];
    	locations[0] = controller.getLocation();
    	locations[1] = location;
    	String[] strings = {robotType};
    	return addMessage(ints, strings, locations);
    }
    public boolean sendNewUnit()
    {
    	int[] ints = new int[2];
    	ints[0] = BroadcastMessage.NEW_UNIT;
    	ints[1] = -1;
    	MapLocation[] locations = new MapLocation[1];
    	locations[0] = controller.getLocation();
    	String[] strings = {controller.getRobotType().toString()};
    	return addMessage(ints, strings, locations);
    }
    public boolean sendFindBlocks(MapLocation fluxLocation, Direction stepDirection, int recepientID)
    {
    	int[] ints = new int[3];
    	ints[0] = BroadcastMessage.FIND_BLOCKS;
    	ints[1] = recepientID;
    	ints[2] = 3;
    	MapLocation[] locations = new MapLocation[1];
    	locations[0] = fluxLocation;
    	String[] strings = null;
    	return addMessage(ints, strings, locations);    
    }
    //follow request archon only
    public boolean sendFollowRequest(MapLocation archonLocation, int archonNumber, int supportUnit)
    {
    	int[] ints = new int[3];
    	ints[0] = BroadcastMessage.FOLLOW_REQUEST;
    	ints[1] = supportUnit;
    	ints[2] = archonNumber;
    	MapLocation[] locations = new MapLocation[1];
    	locations[0] = archonLocation;
    	String[] strings = null;
    	return addMessage(ints, strings, locations);
    }
}
