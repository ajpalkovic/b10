package team153;

public class BroadcastMessage {
	static final int LOW_ENERGON = 1;//LOW_ENERGON (has less than 25% energon) //status, sent when pinged
	static final int UNDER_ATTACK = 2;//UNDER_ATTACK (has been attacked in the past 100 turns) (enemy type, enemy energon level) //broadcasted initially, and sent as status when pinged if has been attacked for past 100 turns
	static final int ENEMY_IN_SIGHT = 3;//(can see an enemy) //broadcasted  (how many enemies and enemy types)
	static final int FLUX_IN_SIGHT = 4;// (can see flux) //broadcasted  (parameter is location)
	static final int NEW_UNIT = 5; //NEW_UNIT (type of unit location on map maybe unique id) //broadcasted
	static final int MAP_INFO  = 6;//broadcasted
	static final int PING = 7;//broadcasted
	static final int PONG = 8;//broadcasted response to ping
	static final int FOLLOW_REQUEST = 9;
	static final int FIND_BLOCKS = 10;
	static final int MOVE = 11; //MOVE BITCH GET OUT DA WAY
    static final int LOW_ALLIED_UNITS = 12;
    static final int SCOUT_ALLIED_UNIT_RELAY = 13;
    static final int SUPPORT = 14;
	   
}
