package yaes.virtualcoordinate;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import agents.DirectionalVCForwarding;
import agents.VCAgent;
import agents.VCMessageHelper;
import yaes.framework.agent.ACLMessage;
import yaes.sensornetwork.constSensorNetwork;
import yaes.sensornetwork.agents.AbstractSensorAgent;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.agents.SensorRoutingHelper;
import yaes.ui.text.TextUi;
import yaes.virtualcoordinate.VCConstants.MessageMode;
import yaes.virtualcoordinate.VCConstants.SinkMobilityMode;
import yaes.virtualcoordinate.VCConstants.SinkMoveNotificationType;
import yaes.world.physical.location.Location;

public class MobileSinkHelper implements Serializable{
	
	public static VCAgent lastBroadcastToEntireNetworkAgent;
	public static VCAgent choosedNeighbor;
	public static VCAgent choosedSinkDestination = null;
	
	public static void moveSink(VCContext context) {
		if(context.getSinkMobilityMode() == SinkMobilityMode.MOBILE) {
			VCAgent sinkAgent = context.getSinkAgent();
			ArrayList<VCAgent> neighbors = sinkAgent.getNeighbors();
			if(context.getSimulationInput().getParameterEnum(VCConstants.SinkMovementType.class).equals(VCConstants.SinkMovementType.RANDOM_NEIGHBOR)) {
				
				int randomIndex = (int)(context.getRandom().nextDouble() * neighbors.size()); // neighbors.size()-1;
				choosedNeighbor = (VCAgent)neighbors.toArray()[randomIndex];
			}
			else if(context.getSimulationInput().getParameterEnum(VCConstants.SinkMovementType.class).equals(VCConstants.SinkMovementType.RANDOM_WAYPOINT)) {
				if(choosedSinkDestination == null || choosedNeighbor.equals(choosedSinkDestination)) {
					List<VCAgent> allAgents = VCMessageHelper.getAllVCAgents(
							context.getWorld(), false);
					boolean farFromAnchors = true;
					int attempts = 0;
					do {
						int randomIndex = (int)(context.getRandom().nextDouble() * allAgents.size());
						choosedSinkDestination = allAgents.get(randomIndex);
						// Check that the new destination is far from anchors which in our case is not near the boundaries of network
						farFromAnchors = true;
						List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
								context.getWorld(), ForwarderSensorAgent.class);
						for (VCAgent anchor  : anchorAgents) {
							double distanceToAnchor = context.getSinkMoveNotifyRadius() * 1.42;
							if(choosedSinkDestination.getVcLocalTable().getNumberOfHops(choosedSinkDestination, anchor) < distanceToAnchor) {
								farFromAnchors = false;
							}
						}
						attempts++;
					} while(!farFromAnchors && attempts < 10);
				}
				DirectionalVCForwarding.useOnlyLocalVCInformation = false;
				choosedNeighbor = DirectionalVCForwarding.getNextHopUsingDVCR(context.getSinkAgent(), choosedSinkDestination, null, context);
				DirectionalVCForwarding.useOnlyLocalVCInformation = true;
				if(choosedNeighbor == null) {
					int randomIndex = (int)(context.getRandom().nextDouble() * neighbors.size()); // neighbors.size()-1;
					choosedNeighbor = (VCAgent)neighbors.toArray()[randomIndex];
				}
				
			}
			sinkAgent.getNode().setLocation(choosedNeighbor.getNode().getLocation());
			context.getSimulationInput().setParameter(constSensorNetwork.SensorDeployment_SinkNodeX, sinkAgent.getNode().getLocation().getX());
			context.getSimulationInput().setParameter(constSensorNetwork.SensorDeployment_SinkNodeY, sinkAgent.getNode().getLocation().getY());
//			TextUi.println(neighbors);
			updateNeighbors(context, sinkAgent);
			// update old neighbors
			for(VCAgent neighbor : neighbors) {
				updateNeighbors(context, neighbor);
			}
			// update new neighbors
			neighbors = sinkAgent.getNeighbors();
			for(VCAgent neighbor : neighbors) {
				updateNeighbors(context, neighbor);
			}
			List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
					context.getWorld(), ForwarderSensorAgent.class);
			for (VCAgent anchor  : anchorAgents) {
				int minHopsToAnchor = Integer.MAX_VALUE;
				for (VCAgent neighbor : neighbors) {
					if(context.getCorrectVCLocalTable().getNumberOfHops(neighbor, anchor) < minHopsToAnchor) {
						minHopsToAnchor = context.getCorrectVCLocalTable().getNumberOfHops(neighbor, anchor);
					}
				}
				sinkAgent.getVcLocalTable().setNumberOfHops(sinkAgent, anchor, minHopsToAnchor+1);
				context.getCorrectVCLocalTable().setNumberOfHops(sinkAgent, anchor, minHopsToAnchor+1);
			}
			sinkAgent.getVcLocalTable().setUpdateTime(System.currentTimeMillis());
		}
	}
	
	@SuppressWarnings("unchecked")
	public static void updateNeighbors(VCContext context, VCAgent agent) {
		agent.setNeighbors((ArrayList<VCAgent>)(ArrayList<?>)SensorRoutingHelper.getNeighbors(agent, context.getWorld()));
	}
	
	public static void SendNotificationOfMove(VCContext context) {
		if(context.getSinkMoveNotifyRadius() == 0 )
			return;
		if(context.getSinkMoveNotificationType() == SinkMoveNotificationType.RUMOR_ROUTING) {
			VCAgent randomNeighbor = VCMessageHelper.getARandomNeighbor(context.getSinkAgent());
			sendNotificationToAgent(context, randomNeighbor, MessageMode.SINK_LOCATION_CHANGE_RANDOM_ROUTING, context.getTTLCount(), null);
		}
		else if(context.getSinkMoveNotificationType() == SinkMoveNotificationType.CIRCLE) {
			double distanceToDestination = Math.abs(
					DirectionalVCForwarding.getL2NormDistance(context.getSinkAgent()
							, lastBroadcastToEntireNetworkAgent, context.getSinkAgent(), context));
			// broadcast inside local area
			if(distanceToDestination - context.getSinkMoveNotifyRadius() < 0) {
				VCAgent randomNeighbor = VCMessageHelper.getARandomNeighbor(context.getSinkAgent());
				sendNotificationToAgent(context, randomNeighbor, MessageMode.SINK_LOCATION_CHANGE_CIRCLE, context.getTTLCount(), lastBroadcastToEntireNetworkAgent);
			} // broadcast to the entire network  
			else {
				sendNotificationToAgent(context, null, MessageMode.SINK_LOCATION_CHANGE_CIRCLE, context.getTTLCount(), null);
				lastBroadcastToEntireNetworkAgent = choosedNeighbor;
			}
			
		}
		
		if(context.getSinkMoveNotificationType() == SinkMoveNotificationType.BROADCAST) {
			double distanceToAreaCenter = Math.abs(
					DirectionalVCForwarding.getL2NormDistance(context.getSinkAgent()
							, lastBroadcastToEntireNetworkAgent, context.getSinkAgent(), context));
			// broadcast inside local area
			if(distanceToAreaCenter - context.getSinkMoveNotifyRadius() < 0) {
				sendNotificationToAgent(context, null, MessageMode.SINK_LOCATION_CHANGE_BROADCAST, context.getSinkMoveNotifyRadius(), lastBroadcastToEntireNetworkAgent);
				
			} // broadcast to the entire network 
			else {
				sendNotificationToAgent(context, null, MessageMode.SINK_LOCATION_CHANGE_BROADCAST, context.getTTLCount(), null);
				lastBroadcastToEntireNetworkAgent = choosedNeighbor;
			}
		} else if(context.getSinkMoveNotificationType() == SinkMoveNotificationType.BROADCAST_DYNAMIC) {
			double distanceToDestination = Math.abs(
					DirectionalVCForwarding.getL2NormDistance(context.getSinkAgent()
							, lastBroadcastToEntireNetworkAgent, context.getSinkAgent(), context));
			if(distanceToDestination - context.getSinkMoveNotifyRadius() < 0) {
				sendNotificationToAgent(context, null, MessageMode.SINK_LOCATION_CHANGE_BROADCAST_DYNAMIC, context.getSinkMoveNotifyRadius(), lastBroadcastToEntireNetworkAgent);
			} else {
				sendNotificationToAgent(context, null, MessageMode.SINK_LOCATION_CHANGE_BROADCAST_DYNAMIC, context.getTTLCount(), null);
				lastBroadcastToEntireNetworkAgent = choosedNeighbor;
			}
		} else {
//			sendNotificationToAgent(context, null, MessageMode.SINK_LOCATION_CHANGE_BROADCAST, 1, lastBroadcastToEntireNetworkAgent);
		}
		
		
	}
	public static void sendNotificationToAgent(VCContext context, VCAgent destination, VCConstants.MessageMode mode, int TTL, VCAgent areaCenterAgent) {
		ACLMessage message = VCMessageHelper.createVCMessage(context, context.getSinkAgent(), destination, context.getSinkAgent().getVcLocalTable(), VCConstants.SinkMobilityMode.MOBILE);
		message.setValue(VCConstants.MODE, mode);
		message.setValue(VCConstants.TTL, TTL);
		message.setValue(VCConstants.AREA_CENTER_AGENT, areaCenterAgent);
		context.getSinkAgent().transmit(message);
	}
}
