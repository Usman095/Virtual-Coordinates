package yaes.virtualcoordinate;

import java.awt.Point;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import yaes.framework.agent.ACLMessage;
import yaes.framework.simulation.IContext;
import yaes.framework.simulation.ISimulationCode;
import yaes.framework.simulation.RandomVariable.Probe;
import yaes.framework.simulation.SimulationInput;
import yaes.framework.simulation.SimulationOutput;
import yaes.sensornetwork.constSensorNetwork;
import yaes.sensornetwork.agents.AbstractSensorAgent;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.agents.SensorRoutingHelper;
import yaes.sensornetwork.model.SensorNetworkMessageConstants;
import yaes.sensornetwork.model.SensorNetworkWorld;
import yaes.sensornetwork.model.SensorNode;
import yaes.sensornetwork.model.constSensorNetworkWorld;
import yaes.ui.text.TextUi;
import agents.BaseStation;
import agents.DirectionalVCForwarding;
import agents.VCAgent;
import agents.VCLocalTable;
import agents.VCMessageHelper;
import agents.VCMobileAgent;

/**
 * 
 * @author Saad Khan
 * 
 */
public class VCSimulation implements Serializable, ISimulationCode, constSensorNetwork, constSensorNetworkWorld, SensorNetworkMessageConstants, VCConstants {
	/**
	 * 
	 */
	private static final long serialVersionUID = -2177386453392188503L;
	long lastPrintTime = 0;
	int sinkMoveCounter = 0;
	int numberOfGeneratedPackets = 0;
	NetworkMode initialMode;
	public static ArrayList<VCAgent> sources = new ArrayList<>();
	public static ArrayList<VCAgent> destinations = new ArrayList<>();
	@Override
	public int update(double time, SimulationInput sip, SimulationOutput sop,
			IContext theContext) {
		VCContext context = (VCContext) theContext;
		SensorNetworkWorld sensorWorld = context.getWorld();
		sensorWorld.setTime((int) time);
		for (SensorNode element : sensorWorld.getSensorNodes()) {
			if (element.isEnabled() && !element.getName().equals("MobileNode")) {
				element.update();
			}
		}
		//sensorWorld.getSinkNode().update();
		if (context.getVisualizer() != null) {
			context.getVisualizer().update();
			try {
				Thread.sleep(10);
			} catch (final InterruptedException e) {
				e.printStackTrace();
			}
		}
		sensorWorld.messageFlush();
		
		long currentTime = System.currentTimeMillis();
		if(currentTime - lastPrintTime > 3000 || VCMessageHelper.getAverageError(context) == 0) {
			printReport(sip, sop, context, sensorWorld, currentTime);
		}
			
		
	
		if(context.getNetworkMode().equals(NetworkMode.VC_INITILIZE_SETUP)) {
			for(int i = 0; i < 100; i++) {
				List<?> allAgents;
				if(context.isForwardVCofAllNodes())
					allAgents = VCMessageHelper.getAllVCAgents(context.getWorld(), false);
				else
					allAgents = VCMessageHelper.getAnchorAgents(context.getWorld(), ForwarderSensorAgent.class);
				int randomIndex = context.getRandom().nextInt(allAgents.size());
				VCAgent agent = (VCAgent)allAgents.get(randomIndex);
				if(!context.isForwardVCofAllNodes() && (!agent.isAnchor() || agent.isInVCMode()))
					continue;
				if(agent.getNeighbors() == null)
					TextUi.println(agent.getName());
				if(context.getRandom().nextDouble() < 10.0/context.getTTLCount()) {
					randomIndex = context.getRandom().nextInt(agent.getNeighbors().size());
					AbstractSensorAgent randomDestination = (AbstractSensorAgent)agent.getNeighbors().toArray()[randomIndex];
					ACLMessage message = VCMessageHelper.createVCMessage(context, agent, (VCAgent)randomDestination, agent.getVcLocalTable(), VCConstants.SinkMobilityMode.STATIC);
					message.setValue(MODE, VCConstants.MessageMode.RUMOR_ROUTING);
	//				message.setValue(FIELD_CONTENT, MESSAGE_DATA);
	//				message.setDestination("*");
	//				agent.updateVirtualCoordinates(message);
					agent.transmit(message);
				}
			}
			//VCContext.setNetworkMode(NetworkMode.INITIALIZEVIRTUALGREEDYFORWARDING);
		}
		
		
		
		
		if(context.getNetworkMode().equals(NetworkMode.DIRECTIONAL_VC)){
			runDVCRToAllNodes(context);
		}
		
		if(context.getNetworkMode() == NetworkMode.DVCR_TO_MOBILE_SINK){
			runDVCRToMobileSink(context);
		}

		//if the average error of the network is zero then 
		//shut the VC process
		if(context.getNetworkMode().equals(NetworkMode.VC_INITILIZE_SETUP)) {
			if(VCMessageHelper.getAverageError(context) != 0)
				return 1;
			else {
				runDVCRToAllNodes(context);
			}
		}
		
		if (context.getNetworkMode().equals(NetworkMode.TCTP)) {
			runTCTP(context);
		}
		
	

		if(context.getNetworkMode().equals(NetworkMode.GREEDY_FORWARDING) || 
				context.getNetworkMode().equals(NetworkMode.DVCR_TO_MOBILE_SINK) || 
				context.getNetworkMode().equals(NetworkMode.DIRECTIONAL_VC) ||
				context.getNetworkMode().equals(NetworkMode.TCTP)) {
			if(context.getSinkMobilityMode() == SinkMobilityMode.MOBILE) {
				sinkMoveCounter++;
				if(sinkMoveCounter % context.getSinkMoveDelay() == 0){
					MobileSinkHelper.moveSink(context);
					MobileSinkHelper.SendNotificationOfMove(context);
				}
			}
			if(sop.getValue(VCConstants.SuccessfullRoutings, Probe.SUM) + sop.getValue(VCConstants.FailedRoutings, Probe.SUM)
					>= sip.getParameterInt(MessagesToBeGenerated) && numberOfGeneratedPackets >= sip.getParameterInt(MessagesToBeGenerated)) {
				printReport(sip, sop, context, sensorWorld, currentTime);
				
				return 0;
			}
			
			return 1;
		}		

		printReport(sip, sop, context, sensorWorld, currentTime);
		return 0;

	}

	boolean firstTime = true;
	public int errorcount = 0;
	private void runTCTP(VCContext context) {
		// TODO Auto-generated method stub
		int predt = 10;
		BaseStation BS = context.getBaseStation();//get the basesatiton
		//get mobileAgent from the world
		AbstractSensorAgent mobileagent = context.getWorld().lookupSensorNodeByName("MobileNode").getAgent();
		TextUi.println("Physical Location of the Node: " + mobileagent.getNode().getLocation().getX() +", "+ mobileagent.getNode().getLocation().getY());
		findMobileVC(context, mobileagent);
		//double[] actualTC = new double[2];
		BS.sample();
		//TextUi.println("Actual TC are: "+actualTC[0] + " " +actualTC[1]);
		for (int i = 0; i < 4; i++) {
			mobileagent.action();
		}
		findMobileVC(context, mobileagent);
		BS.sample();
		double[] predictedTC = new double[2];
		predictedTC = BS.predictTC(predt);
		Shape ellipse = BS.getEllipse(predictedTC[0]+10, predictedTC[1]-10, 50.0, 50.0, BS.getPredictedAngle());
		if (ellipse.contains(predictedTC[0], predictedTC[1]))
			TextUi.println("points are contained");
		if(ellipse.equals(null))
			TextUi.println("ellipse is null");
		TextUi.println("ellipse at: "+predictedTC[0] + " " +predictedTC[1]);
		for (int i = 0; i < predt+8; i++) {
			mobileagent.action();
			if (i >= predt-1-8){
				double[] TC = new double[2];
				findMobileVC(context, mobileagent);
				TC = BS.sample();
				TextUi.println("Actual TC are: "+TC[0] + " " +TC[1]);
				if (ellipse.contains(TC[0], TC[1])) {
					break;
				}
				else {
					if (i == predt-1+8) {
						errorcount++;
					}
				}
			}
		}
		TextUi.println("error count is: "+errorcount);
		
		
		
		//BS.sample();
		//double[] predictedTC = new double[2];
		//if (firstTime) {
			//BS.setPrevTC();
			//firstTime = false;
		//}
		//else
			//predictedTC = BS.predictTC(5);
		
		//Ellipse2D.Double ellipse = BS.getEllipse(predictedTC[0], predictedTC[1], 10.0, 20.0, BS.getPredictedAngle());
		//count++;
		
	}
	
	private void findMobileVC(VCContext context, AbstractSensorAgent mobileagent) {
		
		List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
				context.getWorld(), ForwarderSensorAgent.class);
		List<VCAgent> myAgents = VCMessageHelper.getAllVCAgents(context.getWorld(), false);
		List<VCAgent> nebors = new ArrayList<VCAgent>();
		for (VCAgent agent : myAgents) {
			if (SensorRoutingHelper.isConnected(mobileagent, agent)) {
				nebors.add(agent);
			}
		}
		
		double[] mobileVCs = new double[anchorAgents.size()];
		int i = 0;
		for (VCAgent anchor : anchorAgents) {
			VCLocalTable table = context.getCorrectVCLocalTable();
			double entry = 0;
			for (VCAgent nebor : nebors) {
				//TextUi.println(table.getNumberOfHops(nebor, anchor));
				entry += table.getNumberOfHops(nebor, anchor);
			}
			mobileVCs[i] = entry/nebors.size();
			i++;
		}
		BaseStation BS = context.getBaseStation();
		//give these mobileVCs to basestation to get TCs
		BS.setMobileVC(mobileVCs, anchorAgents.size());
	}


	private void runDVCRToAllNodes(VCContext context) {
//		context.setNetworkMode(NetworkMode.DIRECTIONALVC);
		DirectionalVCForwarding.useOnlyLocalVCInformation = false;
		List<VCAgent> allAgents = VCMessageHelper.getAllVCAgents(context.getWorld(), false);
		List<VCAgent> testAgents = new ArrayList<>();
//		testAgents.add(context.getSinkAgent());
		testAgents.add(VCMessageHelper.getVCAgentByName(context.getWorld(), "S-08"));
		testAgents.add(VCMessageHelper.getVCAgentByName(context.getWorld(), "S-245"));
		for(VCAgent agent1 : allAgents) {
			for(VCAgent agent2 : allAgents) {
				if(agent1.getName().equals(agent2.getName()))
					continue;
				if(context.getRandom().nextDouble() > 0.0001)
					continue;
				if(context.getSimulationOutput().getValue(VCConstants.MessagesGenerated, Probe.SUM) == context.getSimulationInput().getParameterInt(MessagesToBeGenerated)) {
					return;
				}
				sources.add(agent1);
				destinations.add(agent2);
				numberOfGeneratedPackets++;
				ACLMessage newMessage = VCMessageHelper.createVCMessage(context, agent1, agent2, null, VCConstants.SinkMobilityMode.STATIC);
				DirectionalVCForwarding.sendUsingDVCR(agent1, agent2, newMessage, context);
			}
		}
	}
	
	private void runDVCRToMobileSink(VCContext context) {
//		if(numberOfGeneratedPackets > 0 && context.getRandom().nextDouble() > 0.2)
//			return;
		if(numberOfGeneratedPackets > 0 && numberOfGeneratedPackets == context.getSimulationInput().getParameterInt(MessagesToBeGenerated))
			return;
//		context.setNetworkMode(NetworkMode.DIRECTIONALVC);
		DirectionalVCForwarding.useOnlyLocalVCInformation = true;
		List<VCAgent> allAgents = VCMessageHelper.getAllVCAgents(context.getWorld(), false);
		int randomIndex = context.getRandom().nextInt(allAgents.size());
		VCAgent selectedAgent = allAgents.get(randomIndex);
		numberOfGeneratedPackets++;
		ACLMessage newMessage = VCMessageHelper.createVCMessage(context, selectedAgent, context.getSinkAgent(), selectedAgent.getVcLocalTable(), VCConstants.SinkMobilityMode.MOBILE);
		DirectionalVCForwarding.sendUsingDVCR(selectedAgent, context.getSinkAgent(), newMessage, context);
	}
	


	private void printReport( SimulationInput sip, SimulationOutput sop, VCContext context,
			SensorNetworkWorld sensorWorld, long currentTime) {
		TextUi.println("SINK_MOVE_DELAY: " + sip.getParameterInt(SINK_MOVE_DELAY));
		TextUi.println("SINK_MOVE_NOTIFY_RADIOUS: " + sip.getParameterInt(SINK_MOVE_NOTIFY_RADIOUS));
//		TextUi.println("Average Error: " + VCMessageHelper.getAverageError(context));
//		TextUi.println("packets forwarded without VC update: " + VCMessageHelper.getAverageForwardedMessagesWithoutVCUpdate(context));
//		TextUi.println("Number of Nodes in VC mode: " + VCMessageHelper.getNumberOfNodesInVCMode(context));
		TextUi.println("Messages Sent: " + sop.getValue(
		SENSORNETWORK_MESSAGES_SENT, Probe.SUM) + " Average messages per node: " + (double)sop.getValue(
				SENSORNETWORK_MESSAGES_SENT, Probe.SUM)/
				VCMessageHelper.getAllVCAgents(context.getWorld(), false).size());
		TextUi.println("Messages Generated to be routed: " + numberOfGeneratedPackets );
		TextUi.println("All Messages Generated: " + sop.getValue(VCConstants.MessagesGenerated, Probe.SUM) );
		TextUi.println("Successfull Routings: " + sop.getValue(VCConstants.SuccessfullRoutings, Probe.SUM) + 
				" Failed Routings: " + sop.getValue(VCConstants.FailedRoutings, Probe.SUM) + 
				" Equation Solve fails: " + DirectionalVCForwarding.solveEquationFails);
		TextUi.println("Routed Messages: " + sop.getValue(VCConstants.RoutedMessages, Probe.SUM));
		TextUi.println("Average Path Length: " + sop.getValue(VCConstants.RoutedMessages, Probe.SUM)/sop.getValue(
				VCConstants.SuccessfullRoutings, Probe.SUM));
//		TextUi.println(sources.size());
//		if(sources.size() < 20) {
//			for(int i = 0; i < VCSimulation.sources.size(); i++) {
//				TextUi.println(sources.get(i));
//				TextUi.println(destinations.get(i));
//				TextUi.println("------------");
//			}
//		}
//			TextUi.println("Energy: " + sop.getValue(
//					SENSORNETWORK_TRANSMISSION_ENERGY, Probe.SUM));
//			TextUi.println("Messages Overheard: " + sop.getValue(
//					SENSORNETWORK_MESSAGES_OVERHEARD, Probe.SUM));
//			List<AbstractSensorAgent> allAgents = SensorRoutingHelper
//					.getSensorAgents(sensorWorld, ForwarderSensorAgent.class);
//			for(AbstractSensorAgent agent : allAgents) {
//				TextUi.println(agent);
//			}
		lastPrintTime = currentTime;
	}


	/**
	 * Collect data about the data.
	 */
	@Override
	public void postprocess(SimulationInput sip, SimulationOutput sop,
			IContext theContext) {
		sop.update(Metrics_TransmissionEnergySum, sop.getValue(
				SENSORNETWORK_TRANSMISSION_ENERGY, Probe.SUM));
		sop.update(Metrics_MessagesSentSum, sop.getValue(
				SENSORNETWORK_MESSAGES_SENT, Probe.SUM));
		sop.update(VCConstants.Metrics_MessagesGeneratedSum, sop.getValue(
				VCConstants.MessagesGenerated, Probe.SUM));
		sop.update(VCConstants.Metrics_SuccessfullRoutings, sop.getValue(
				VCConstants.SuccessfullRoutings, Probe.SUM));
		sop.update(VCConstants.Metrics_RoutedMessages, sop.getValue(
				VCConstants.RoutedMessages, Probe.SUM));
		sop.update(VCConstants.Metrics_AveragePathLength, (double)sop.getValue(
				VCConstants.RoutedMessages, Probe.SUM)/sop.getValue(
						VCConstants.SuccessfullRoutings, Probe.SUM));
		
		sop.update(VCConstants.Metrics_FailedRoutings, sop.getValue(
				VCConstants.FailedRoutings, Probe.SUM));
		sop.update(VCConstants.Metrics_AverageNeighborsCount, VCContext.averageNeighborsCount);
	}

	@Override
	public void setup(SimulationInput sip, SimulationOutput sop,
			IContext theContext) {
		final VCContext context = (VCContext) theContext;
		context.initialize(sip, sop);
		this.initialMode = context.getNetworkMode();
//		SensorRoutingHelper.createPathsForForwarderSensorAgents(
//                context.getSinkAgent(), context.getWorld());

	}
	

}
