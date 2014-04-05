package yaes.virtualcoordinate; 
import yaes.sensornetwork.constSensorNetwork;

/**
 * Defines the additional constant required by the
 * Virtual Coordinate system
 * @author Saad Khan
 *
 */
public interface VCConstants extends constSensorNetwork{
	public enum CSUBenchmark{
		CIRCULAR3VOID, ODD, SPIRAL, SPARSEGRID, BUILDING,
		CONCAVEVOID1, CONCAVEVOID2, TESTCIRCULAR3VOID, UNIFORM_GRID
	}
	
	public enum NetworkMode{
		VC_INITILIZE_SETUP, DIRECTIONAL_VC, DVCR_TO_MOBILE_SINK, TCTP, 
		GREEDY_FORWARDING, DIJKSTRA, PERIMETER_ROUTING, ROUTING_COMPLETE,
	}
	
	public enum MessageMode {
		RUMOR_ROUTING, DIRECTIONAL_VC, SINK_LOCATION_CHANGE_BROADCAST, SINK_LOCATION_CHANGE_BROADCAST_DYNAMIC , SINK_LOCATION_CHANGE_RANDOM_ROUTING, SINK_LOCATION_CHANGE_CIRCLE
	}
	
	public enum AnchorPlacementMode {
		RANDOM, EXTREMES
	}
	
	public enum SinkMobilityMode {
		STATIC, MOBILE,
	}
	
	public enum SinkMovementType {
		RANDOM_NEIGHBOR, RANDOM_WAYPOINT,
	}
	
	public enum SinkMoveNotificationType {
		BROADCAST, BROADCAST_DYNAMIC, RUMOR_ROUTING, CIRCLE
	}
	
	public static final String MOBILITY_MODE = "Mobility mode";
	public static final String VCSETUP = "VCSETUPMODE"; 
	public static final String TTL = "TTL";
	public static final String PREVIOUS_AGENT = "PREVIOUS_AGENT";
	public static final String IN_PATH_AGENTS = "IN_PATH_AGENTS";
	public static final String SENDER_AGENT = "SENDER_AGENT";
	public static final String SINK_AGENT = "SINK_AGENT";
	public static final String PATH_LENGTH = "PATH_LENGTH";
	public static final String VC_LOCAL_TABLE = "VC_LOCAL_TABLE";
	public static final String TTLCount = "TTLCount";
	public static final String SINK_MOVE_NOTIFY_RADIOUS = "SINK_MOVE_NOTIFY_RADIOUS";
	public static final String SINK_MOVE_DELAY = "SINK_MOVE_DELAY";
	public static final String AREA_CENTER_AGENT = "AREA_CENTER_AGENT";
	public static final String Metrics_SuccessfullRoutings = "Successfull routings";
	public static final String SuccessfullRoutings = "SuccessfullRoutings";
	public static final String Metrics_FailedRoutings = "Failed routings";
	public static final String FailedRoutings = "FailedRoutings";
	public static final String Metrics_MessagesGeneratedSum = "Number of generated messages";
	public static final String Metrics_RoutedMessages = "Routed messages";
	public static final String Metrics_AveragePathLength = "Average path length";
	public static final String RoutedMessages = "RoutedMessages";
	public static final String MessagesToBeGenerated = "MessagesToBeGenerated";
	public static final String MessagesGenerated = "MessagesGenerated";
	public static final String NetworkScale = "NetworkScale";
	public static final String AnchorCount = "AnchorCount";
	public static final String NetworkDensity = "NetworkDensity";
	public static final String UseDensity = "UseDensity";
	public static final String DIRECTIONALVC = "DIRECTIONALVCROUTING";
	public static final String Metrics_AverageNeighborsCount = "Average Neighbors Count";
}
