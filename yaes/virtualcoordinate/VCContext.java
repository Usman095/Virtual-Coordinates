package yaes.virtualcoordinate;

import java.awt.Color;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.File;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import util.ScanFile;
import util.PaintVCNode;
import yaes.framework.simulation.AbstractContext;
import yaes.framework.simulation.SimulationInput;
import yaes.framework.simulation.SimulationOutput;
import yaes.sensornetwork.Environment;
import yaes.sensornetwork.constSensorNetwork;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.agents.SensorRoutingHelper;
import yaes.sensornetwork.energymodel.RapaportCommunicationEnergyModel;
import yaes.sensornetwork.model.SensorNetworkWorld;
import yaes.sensornetwork.model.SensorNode;
import yaes.sensornetwork.model.SinkNode;
import yaes.sensornetwork.visualization.SensorNetworkWorldPainter;
import yaes.sensornetwork.visualization.paintSensorNode;
import yaes.ui.text.TextUi;
import yaes.ui.visualization.Visualizer;
import yaes.ui.visualization.painters.IPainter;
import yaes.ui.visualization.painters.paintMobileNode;
import yaes.util.SaveLoadUtil;
import yaes.world.physical.location.IMoving;
import yaes.world.physical.location.Location;
import yaes.world.physical.map.ArrangementHelper;
import agents.BaseStation;
import agents.VCAgent;
import agents.VCLocalTable;
import agents.VCMessageHelper;
import agents.VCMobileAgent;
import environment.VCEnvironmentHelper;

/**
 * Defines the context of the Virtual Coordinate Sensor Network
 * 
 * @author Saad Khan
 * 
 */
public class VCContext extends AbstractContext implements VCConstants, Serializable {

	private static final long serialVersionUID = 5649203748203504717L;
	private int sensorNodeCount;
	private boolean visualize = true;
	private boolean forwardVCofAllNodes = true;
	private Visualizer visual;
	static VCAgent sinkAgent = null;
	static VCAgent mobileTargetAgent = null;
	protected SensorNetworkWorld sensorWorld;
	protected double transmissionRange;
	private List<Location> locationList;
	private int anchorCount = 0;
	private int messagesInNetworkCount = 0;
	private int TTLCount;
	private NetworkMode networkMode;
	private Rectangle2D.Double overallInterestRectangle;
	private SinkMobilityMode sinkMobilityMode;
	private int sinkMoveDelay;
	private SinkMoveNotificationType sinkMoveNotificationType;
	private ArrayList<String> pathRecord = new ArrayList<String>();
	private int sinkMoveNotifyRadius;
	public static int averageNeighborsCount;
	private BaseStation BS;
	@SuppressWarnings("unchecked")
	@Override
	public void initialize(SimulationInput sip, SimulationOutput sop) {
		super.initialize(sip, sop);
		pathRecord =  new ArrayList<String>();
		double interestRectangleX = sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleX);
		double interestRectangleY = sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleY);
		double interestRectangleWidth = sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleWidth);
		// In current application we with and height are the same
		double interestRectangleHeight = sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleWidth);
		overallInterestRectangle = new Rectangle2D.Double(interestRectangleX,
				interestRectangleY, interestRectangleWidth,
				interestRectangleHeight);

		// end of world creation
		
		
//		locationList = SensorNodeGenerator.generateNodes(sip,
//				sip.getParameterDouble(VCConstants.NetworkSale));
//		locationList = SensorNodeGenerator.generateNodes(sip,
//		sip.getParameterDouble(VCConstants.NetworkSale));
//sip.setParameter(SensorDeployment_SensorNodeCount,
//		locationList.size());
//sop.createVariable(VCConstants.MessagesGenerated, false);
		this.networkMode = sip.getParameterEnum(NetworkMode.class);
		this.sinkMobilityMode = sip.getParameterEnum(SinkMobilityMode.class);
		this.sinkMoveNotificationType = sip.getParameterEnum(SinkMoveNotificationType.class);
		//The sensor node count should be in accordance with the benchmark
		if(!sip.getParameterEnum(SensorArrangement.class).equals(SensorArrangement.BENCHMARK)) {
			if(getSimulationInput().getParameterInt(UseDensity) == 0) {
				this.sensorNodeCount = sip
						.getParameterInt(constSensorNetwork.SensorDeployment_SensorNodeCount);
			} else {
				this.sensorNodeCount = (int)	// In our case, with and height needed to be the same.
						(sip.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleWidth)
						* sip.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleWidth)
						* sip.getParameterDouble(VCConstants.NetworkDensity));
			}
		}
		this.visualize = (sip.getParameterEnum(VisualDisplay.class) == VisualDisplay.YES);
		this.transmissionRange = sip
				.getParameterDouble(SensorDeployment_TransmissionRange);
		this.theWorld = sensorWorld;
		this.random = new Random(
				sip.getParameterInt(Scenario_IntruderMovementRandomSeed));
		this.TTLCount = sip.getParameterInt(VCConstants.TTLCount);
		this.setSinkMoveNotifyRadius(sip.getParameterInt(VCConstants.SINK_MOVE_NOTIFY_RADIOUS));
		this.setSinkMoveDelay(sip.getParameterInt(VCConstants.SINK_MOVE_DELAY));
		this.anchorCount = sip.getParameterInt(AnchorCount);
		if (sip.getParameterDouble(NetworkScale) > Math.sqrt(4))
			anchorCount = 40;
		else if (sip.getParameterDouble(NetworkScale) > Math.sqrt(2))
			anchorCount = 20;
		//setting up the network mode as virtual coordinate setup phase
		//SensorRoutingHelper.createPathsForForwarderSensorAgents(
		// sinkNode.getAgent(), sensorWorld);
		SaveLoadUtil<SensorNetworkWorld> sluSN = new SaveLoadUtil<>();
		// initilizes where the cache file will go
		File sensorWorldCacheFile = new File(VirtualCoordinateMain.cache.getPath()
				+ "/cache.sensorworld.benchmark=" + sip.getParameterEnum(CSUBenchmark.class).name()
				+ ".scale=" + sip.getParameterDouble(NetworkScale)
				+ ".nodes=" + sip.getParameterInt(SensorDeployment_SensorNodeCount)
				+ ".width=" + sip.getParameterDouble(SensorDeployment_InterestRectangleWidth)
				+ ".height=" + sip.getParameterDouble(SensorDeployment_InterestRectangleHeight)
				+ ".density=" + sip.getParameterDouble(NetworkDensity)
				+ ".arrangement=" + sip.getParameterEnum(SensorArrangement.class));
		if (sensorWorldCacheFile.exists()) {
			// loads the cache file into the sensor world, and changes the SO into the new one
			sensorWorld = sluSN.load(sensorWorldCacheFile);
			sensorWorld.changeSimulationOutput(sop);
			this.theWorld = sensorWorld;
		} else {
			createWorld(sip, sop);
			sluSN.save(sensorWorld, sensorWorldCacheFile);
		}
		List<VCAgent> allAgents = VCMessageHelper.getAllVCAgents(getWorld(), true);
		sinkAgent = (VCAgent)sensorWorld.getSinkNode().getAgent();
		averageNeighborsCount = 0;
		for (VCAgent agent : allAgents) {
			VCAgent vcAgent =  agent;
			vcAgent.setNeighbors((ArrayList<VCAgent>)(ArrayList<?>)SensorRoutingHelper.getNeighbors(vcAgent, getWorld()));
			vcAgent.setContext(this);
			vcAgent.getContext().setForwardVCofAllNodes(this.forwardVCofAllNodes);
			averageNeighborsCount += vcAgent.getNeighbors().size();
//			if(vcAgent.isAnchor())
//				TextUi.println(vcAgent);
			//vcAgent.setVcLocalTable(new VCLocalTable());
			if(vcAgent.getNeighbors().size() == 0)
				continue;
			int randomIndex = this.random.nextInt(vcAgent.getNeighbors().size());
			//selects one node randomly from its neighbors

			VCAgent randomDestination = (VCAgent) vcAgent
					.getNeighbors().toArray()[randomIndex];
			( agent).setForwardingDestination(randomDestination
					.getName());
		}
		
		if(this.getNetworkMode() == NetworkMode.DVCR_TO_MOBILE_SINK) {
			// move sink to one of its neighbors
			MobileSinkHelper.choosedSinkDestination = null;
			MobileSinkHelper.moveSink(this);
			MobileSinkHelper.lastBroadcastToEntireNetworkAgent = MobileSinkHelper.choosedNeighbor;
		}
		
		if(this.getNetworkMode() == NetworkMode.TCTP) {
			BS = new BaseStation(sip, this);
			final SensorNode Node = new SensorNode();
			Node.setName("MobileTarget");
			mobileTargetAgent = createSensorNodeAgent(sip, Node, false);
			Node.setAgent(mobileTargetAgent);
			sinkAgent.setNode(Node);
			sensorWorld.addSensorNode(Node);
			sensorWorld.getDirectory().addAgent(mobileTargetAgent);
		}

		averageNeighborsCount /= allAgents.size();
		TextUi.println("Agents count: " + allAgents.size());
		TextUi.println("Anchors count: " + getAnchorCount());
		TextUi.println("TTL count: " + getTTLCount());
		TextUi.println("Average number of neigbors: " + averageNeighborsCount);
		TextUi.println("NetworkMode: " + getNetworkMode());
		
		// create the visual representations
		if (visualize) {
			createVisualRepresentation(null);
		}
	}

	/**
	 * This method creates the world. This is an expensive method due to the
	 * expense of the shortest path creation. For certain iterations it is
	 * convenient to cache it.
	 * 
	 * FIXME: make it return the sensor world, not the list of agents
	 * 
	 * @param sip
	 * @param sop
	 * @return
	 */
	private void createWorld(SimulationInput sip, SimulationOutput sop) {
		sensorWorld = new SensorNetworkWorld(sop);
		sensorWorld.setEndOfTheWorldTime((int) sip.getStopTime() - 1);
		
		ScanFile parser = null;
		switch (sip.getParameterEnum(CSUBenchmark.class)) {
		case CIRCULAR3VOID:
			parser = new ScanFile("datasets/CircularVoidNetwork.txt", sip);
			break;
		case BUILDING:
			parser = new ScanFile("datasets/Building.txt", sip);
			break;
		case CONCAVEVOID1:
			parser = new ScanFile("datasets/Concave1.txt", sip);
			break;
		case CONCAVEVOID2:
			parser = new ScanFile("datasets/Concave2.txt", sip);
			break;
		case ODD:
			parser = new ScanFile("datasets/OddNetwork.txt", sip);
			break;
		case SPARSEGRID:
			parser = new ScanFile("datasets/SparseGrid.txt", sip);
			break;
		case SPIRAL:
			parser = new ScanFile("datasets/SpiralNetwork.txt", sip);
			break;
		case TESTCIRCULAR3VOID:
			parser = new ScanFile("datasets/testCircularVoidNetwork.txt", sip);
			break;
		default:
			break;

		}
		try {
			 locationList = parser.processLocationList();
//			 sip.setParameter(SensorDeployment_SensorNodeCount, locationList.size());
			 sop.createVariable(VCConstants.MessagesGenerated, false);
			 sop.createVariable(VCConstants.SuccessfullRoutings, false);
			 sop.createVariable(VCConstants.RoutedMessages, false);
			 sop.createVariable(VCConstants.FailedRoutings, false);
			 this.sensorNodeCount = locationList.size();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//The sensor node count should be in accordance with the benchmark
		if(!sip.getParameterEnum(SensorArrangement.class).equals(SensorArrangement.BENCHMARK)) {
			if(getSimulationInput().getParameterInt(UseDensity) == 0) {
				this.sensorNodeCount = sip
						.getParameterInt(constSensorNetwork.SensorDeployment_SensorNodeCount);
			} else {
				this.sensorNodeCount = (int)	// In our case, with and height needed to be the same.
						(sip.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleWidth)
						* sip.getParameterDouble(constSensorNetwork.SensorDeployment_InterestRectangleWidth)
						* sip.getParameterDouble(VCConstants.NetworkDensity));
			}
		}
		
		
		
		//Creation of sensor nodes. Here each sensor ndoe is 
		//attached to an agent and certain properties may also be set
		for (int i = 0; i < sensorNodeCount; i++) {
			final SensorNode staticNode = new SensorNode();
			staticNode.setName("S-" + String.format("%02d", i));
			if(sip.getParameterEnum(SensorArrangement.class).equals(SensorArrangement.BENCHMARK))
				staticNode.setLocation(locationList.get(i));
			final VCAgent staticNodeAgent = createSensorNodeAgent(
					sip, staticNode, false);
			// set the energy model
			RapaportCommunicationEnergyModel cem = new RapaportCommunicationEnergyModel(
					RapaportCommunicationEnergyModel.PowerConsumptionScenario.HIGH_PATH_LOSS);
			staticNodeAgent.setEnergyParameters(cem, 100, 0, true);
			// with this probability the node would be an anchor
			staticNode.setAgent(staticNodeAgent);
			staticNodeAgent.setNode(staticNode);
			sensorWorld.addSensorNode(staticNode);
			sensorWorld.getDirectory().addAgent(staticNodeAgent);
		}
		
//TODO: Add the Mobile Agent
		Location src = new Location(10, 10);
		Location dest = new Location(100, 100);

		final SensorNode mobileNode = new SensorNode();
		mobileNode.setName("MobileNode");
		mobileNode.setLocation(src);
		final VCMobileAgent mobileNodeAgent = new VCMobileAgent(mobileNode.getName(), sensorWorld);
		//The following properties maybe skipped as desired
		mobileNodeAgent.setTransmissionRange(sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_TransmissionRange));
		mobileNodeAgent.setSensorRange(sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_SensorRange));
		RapaportCommunicationEnergyModel cem = new RapaportCommunicationEnergyModel(
				RapaportCommunicationEnergyModel.PowerConsumptionScenario.HIGH_PATH_LOSS);
		mobileNodeAgent.setEnergyParameters(cem, 100, 0, true);
		mobileNodeAgent.setStartLocation(src);
		mobileNodeAgent.setTerminalLocation(dest);
		//adding mobile agent on the node 
		mobileNode.setAgent(mobileNodeAgent);
		//adding node properties to mobile agent
		mobileNodeAgent.setNode(mobileNode);
		//adding the node and its agent to the sensor world
		sensorWorld.addSensorNode(mobileNode);
		sensorWorld.getDirectory().addAgent(mobileNodeAgent);
//TODO: This ends the implementation of mobile agent and its node
		

		
		//distribution of the sensor nodes if SensorArrangement is not Benchmark
		distributeSensorNodes(sip);
		
		
		
		
		//creation of the sink node
		SinkNode sinkNode = new SinkNode();
		sinkNode.setName("Sink");
//		if(sip.getParameterEnum(SensorArrangement.class) == SensorArrangement.BENCHMARK)
//			sinkNode.setLocation(locationList.get(7));
//		else {
			double sinkNodeX = sip.getParameterDouble(constSensorNetwork.SensorDeployment_SinkNodeX);
			double sinkNodeY = sip.getParameterDouble(constSensorNetwork.SensorDeployment_SinkNodeY);
			sinkNode.setLocation(new Location(sinkNodeX, sinkNodeY));
//		}
		
		//select anchors
		selectAnchors();
		
		sinkAgent = createSensorNodeAgent(
				sip, sinkNode, false);
		sinkNode.setAgent(sinkAgent);
		sinkAgent.setNode(sinkNode);
		sensorWorld.setSinkNode(sinkNode);
		

		// now we need to call something to create the routes
		TextUi.println("Calculating paths...");
		VCMessageHelper.setShortestPathToAnchors(this);
		
	}


	/**
	 * Distributes the sensor nodes according to the specification
	 * 
	 * @param sip
	 */
	private void distributeSensorNodes(SimulationInput sip) {
		ArrayList<IMoving> list2 = new ArrayList<IMoving>(sensorWorld
				.getSensorNodes());
		switch (sip.getParameterEnum(SensorArrangement.class)) {
		case GRID: {
			ArrangementHelper
					.arrangeInAGrid(
							(int) overallInterestRectangle.x,
							(int) overallInterestRectangle.y,
							(int) (overallInterestRectangle.x + overallInterestRectangle.width),
							(int) (overallInterestRectangle.y + overallInterestRectangle.height),
							list2);
			break;
		}
		case GRID_WITH_NOISE: {
			ArrangementHelper
					.arrangeInAGridWithNoise(
							(int) overallInterestRectangle.x,
							(int) overallInterestRectangle.y,
							(int) (overallInterestRectangle.x + overallInterestRectangle.width),
							(int) (overallInterestRectangle.y + overallInterestRectangle.height),
							list2, random, 0.1);
			break;
		}
		case RANDOM: {
			ArrangementHelper.arrangeRandomlyInARectangle(
					overallInterestRectangle, list2, random);
			break;
		}
		}
		
	}
	
	private void selectAnchors() {
		switch (sip.getParameterEnum(AnchorPlacementMode.class)) {
		case EXTREMES: {
			List<SensorNode> sensorNodes = getWorld().getSensorNodes();
			List<SensorNode> anchors = new ArrayList<SensorNode>();
			double max_distance = Integer.MIN_VALUE;
			for(SensorNode sensorNode1 : sensorNodes) {
				for(SensorNode sensorNode2 : sensorNodes) {
					if(sensorNode1.getLocation().distanceTo(sensorNode2.getLocation()) > max_distance) {
						max_distance = sensorNode1.getLocation().distanceTo(sensorNode2.getLocation());
						anchors.clear();
						anchors.add(sensorNode1);
						anchors.add(sensorNode2);
					}
				}
			}
			VCMessageHelper.getVCAgentByName(getWorld(), anchors.get(0).getName()).setAnchor(true);
			VCMessageHelper.getVCAgentByName(getWorld(), anchors.get(1).getName()).setAnchor(true);
			int nodesCountToBeAdded = anchorCount - anchors.size();
			for (int i = 0; i < nodesCountToBeAdded; i++) {
				SensorNode max_distance_node = sensorNodes.get(0);
				max_distance = Integer.MIN_VALUE;
				for(SensorNode sensorNode : sensorNodes) {
					double distance = 0;
					for(SensorNode anchor: anchors) {
						distance += sensorNode.getLocation().distanceTo(anchor.getLocation());
					}
					if(distance > max_distance && !VCMessageHelper.getVCAgentByName(getWorld(), sensorNode.getName()).isAnchor()) {
						max_distance = distance;
						max_distance_node = sensorNode;
					}
				}
				anchors.add(max_distance_node);
				VCMessageHelper.getVCAgentByName(getWorld(), max_distance_node.getName()).setAnchor(true);
			}
			break;
		} case RANDOM: {
			ArrayList<Integer> anchorIndexes = new ArrayList<>(); 
			while(true) {
				int randomNodeIndex = random.nextInt(sensorNodeCount);
				boolean repetitive = false;
				for(int index: anchorIndexes) {
					if(index == randomNodeIndex) {
						repetitive = true;
					}
				}
				if(!repetitive) {
					anchorIndexes.add(randomNodeIndex);
					VCMessageHelper.getAllVCAgents(sensorWorld, false).get(randomNodeIndex).setAnchor(true);
				}
				if(anchorIndexes.size() == anchorCount)
					break;
			}
			break;
		}
		}
				
	}
	
	public void applyMessagePassingAlgorithm(SimulationInput sip) {

	}
	
	// perform a transformation of the location from the Colorado State
	// DB to a more
	// realistic metric - later we can put an affine transform here
	private Location transromLocation(Location spec) {
		Point2D.Double pointColState = spec.asPoint();
		Point2D.Double pointYaes = new Point2D.Double();
		// create an affine transform which performs the transformation
		AffineTransform at = AffineTransform.getScaleInstance(10, 10);
		at.concatenate(AffineTransform.getRotateInstance(Math.PI * 0.2));
		at.transform(pointColState, pointYaes);
		Location transformedLocation = new Location(pointYaes.x, pointYaes.y);
		return transformedLocation;
	}

	/**
	 * Creates a visual representation
	 * 
	 * @param sip
	 */
	@Override
	public void createVisualRepresentation(Visualizer existingVisualizer) {
		//
		// create the visualizer covering the full considered area
		//
		if (existingVisualizer != null) {
			visualizer = existingVisualizer;
			visualizer.removeAllObjects();
		} else {
			if (sip.getSimulationControlPanel() == null) {
				visualizer = new Visualizer(1000, 1000, null, "Sensor network",
						true);
			} else {
				visualizer = new Visualizer(1000, 1000, null, "Sensor network",
						true, true);
				sip.getSimulationControlPanel().addTab("Visual", visualizer);
			}
		}
		
		
		// add the painting for the sensor nodes
		// IPainter painterNode = new paintSensorNode(sensorWorld);
		IPainter painterNode = new PaintVCNode(sensorWorld);
		for (final SensorNode node : sensorWorld.getSensorNodes()) {
			visualizer.addObject(node, painterNode);
		}
		// add the painting of the sink node
		visualizer.addObject(sensorWorld.getSinkNode(), new paintMobileNode(20,
				Color.black, Color.red));
		// add the painting for the sensor world
		visualizer.addObject(sensorWorld, new SensorNetworkWorldPainter());
		visualizer.getVisualizationProperties().setPropertyValue(
				paintSensorNode.vpropPaintRoutes, new Boolean(true));
	}

	protected Environment createEnvironment(SimulationInput sip,
			SimulationOutput sop) {
		Environment retval = VCEnvironmentHelper.generateVCEnvironment();
		return retval;
	}

	public VCAgent createSensorNodeAgent(SimulationInput sip,
			SensorNode staticNode, boolean isAnchor) {
		final VCAgent agent = new VCAgent(staticNode.getName(), this);
		agent.setTransmissionRange(sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_TransmissionRange));
		agent.setSensorRange(sip
				.getParameterDouble(constSensorNetwork.SensorDeployment_SensorRange));
		agent.setAnchor(isAnchor);
		return agent;
	}

	public Visualizer getVisual() {
		return visual;
	}

	@Override
	public SensorNetworkWorld getWorld() {
		return sensorWorld;
	}

	public VCAgent getSinkAgent() {
		return sinkAgent;
	}

	public int getMessagesInNetworkCount() {
		return messagesInNetworkCount;
	}

	public void setMessagesInNetworkCount(int messagesInNetworkCount) {
		this.messagesInNetworkCount = messagesInNetworkCount;
	}

	public int getAnchorCount() {
		return anchorCount;
	}

	public int getTTLCount() {
		return TTLCount;
	}

	public int getSensorNodeCount() {
		return sensorNodeCount;
	}

	public void setTTLCount(int tTLCount) {
		TTLCount = tTLCount;
	}
	
	public VCLocalTable getCorrectVCLocalTable() {
		return getSinkAgent().getCorrectVCLocalTable();
	}

	public NetworkMode getNetworkMode() {
		return networkMode;
	}

	public void setNetworkMode(NetworkMode networkMode) {
		this.networkMode = networkMode;
	}

	public void setCorrectVCLocalTable(VCLocalTable correctVCLocalTable) {
		getSinkAgent().setCorrectVCLocalTable(correctVCLocalTable);
	}

	public ArrayList<String> getPathRecord() {
		return pathRecord;
	}

	public void setPathRecord(ArrayList<String> pathRecord) {
		this.pathRecord = pathRecord;
	}

	public boolean isForwardVCofAllNodes() {
		return forwardVCofAllNodes;
	}

	public void setForwardVCofAllNodes(boolean forwardVCofAllNodes) {
		this.forwardVCofAllNodes = forwardVCofAllNodes;
	}

	public SinkMobilityMode getSinkMobilityMode() {
		return sinkMobilityMode;
	}

	public void setSinkMobilityMode(SinkMobilityMode sinkMobilityMode) {
		this.sinkMobilityMode = sinkMobilityMode;
	}

	public int getSinkMoveNotifyRadius() {
		return sinkMoveNotifyRadius;
	}

	public void setSinkMoveNotifyRadius(int sinkMoveNotifyRadius) {
		this.sinkMoveNotifyRadius = sinkMoveNotifyRadius;
	}

	public int getSinkMoveDelay() {
		return sinkMoveDelay;
	}

	public void setSinkMoveDelay(int sinkMoveDelay) {
		this.sinkMoveDelay = sinkMoveDelay;
	}

	public SinkMoveNotificationType getSinkMoveNotificationType() {
		return sinkMoveNotificationType;
	}

	public void setSinkMoveNotificationType(SinkMoveNotificationType sinkMoveNotificationType) {
		this.sinkMoveNotificationType = sinkMoveNotificationType;
	}

}
