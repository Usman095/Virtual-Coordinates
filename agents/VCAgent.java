package agents;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import yaes.framework.agent.ACLMessage;
import yaes.sensornetwork.constSensorNetwork;
import yaes.sensornetwork.agents.AbstractSensorAgent;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.model.Perception;
import yaes.ui.format.Formatter;
import yaes.virtualcoordinate.MobileSinkHelper;
import yaes.virtualcoordinate.VCConstants;
import yaes.virtualcoordinate.VCContext;

/**
 * Defines the agent for the virtual coordinate setup
 *        
 * @author Saad Khan
 * 
 */  
public class VCAgent extends ForwarderSensorAgent implements Serializable,
		VCConstants {

	private static final long serialVersionUID = 5786456369198875212L;
	private boolean isAnchor = false;
	private boolean isInPreviousLocalArea = false;
	private boolean isInLocalArea = false;
	private VCLocalTable vcLocalTable = new VCLocalTable();
	private VCLocalTable correctVCLocalTable = new VCLocalTable();
	private ArrayList<VCAgent> neighbors;
	private int messagesForwardedWithoutVCUpdate = 0;
	private boolean isInVCMode = false;
	private VCContext context;
	VCAgent nextAgentToSink = null;

	@Override
	protected void handleReceivedMessage(final ACLMessage message) {
		VCAgent areaCenterAgent = null;
		double distanceToAreaCenter = 0;
		boolean vCTableUpdated = false;
		boolean doBroadcast = false;
		message.setValue(VCConstants.PATH_LENGTH,
				(int) message.getValue(VCConstants.PATH_LENGTH) + 1);
		switch ((VCConstants.MessageMode) (message.getValue(MODE))) {
		case SINK_LOCATION_CHANGE_BROADCAST:
			areaCenterAgent = (VCAgent) message
					.getValue(VCConstants.AREA_CENTER_AGENT);
			if (areaCenterAgent != null) {
				distanceToAreaCenter = Math.abs(DirectionalVCForwarding
						.getL2NormDistance(areaCenterAgent, this,
								areaCenterAgent, context));
			} else {
				distanceToAreaCenter = Math.abs(DirectionalVCForwarding
						.getL2NormDistance(this, this,
								MobileSinkHelper.lastBroadcastToEntireNetworkAgent, context));
			}
			// update the VC table. If there is no change, leave.
			vCTableUpdated = updateVCTableOfNewSinkLocation(message); 
			if (!vCTableUpdated) {
				break;
			}
			// broadcast to the entire network until TTL becomes 0
			if(areaCenterAgent == null && (int) message
							.getValue(VCConstants.TTL) > 0) {
				doBroadcast = true;
				if(isInLocalArea())
					setInPreviousLocalArea(true);
				else
					setInPreviousLocalArea(false);
				if(distanceToAreaCenter < context
						.getSinkMoveNotifyRadius()) {
					setInLocalArea(true);
				} else {
					setInLocalArea(false);
				}
			}
			// broadcast just inside the local area
			if(areaCenterAgent != null && distanceToAreaCenter < context
					.getSinkMoveNotifyRadius()) {
				doBroadcast = true;
				setInLocalArea(true);
			}
			if (doBroadcast) {
				ACLMessage newMessage = VCMessageHelper.createVCMessage(
						context, this, null, this.getVcLocalTable(),
						VCConstants.SinkMobilityMode.MOBILE);
				newMessage.setValue(constSensorNetwork.MODE,
						VCConstants.MessageMode.SINK_LOCATION_CHANGE_BROADCAST);
				newMessage.setValue(VCConstants.AREA_CENTER_AGENT,
						areaCenterAgent);
				newMessage.setValue(VCConstants.TTL,
						(int) message.getValue(VCConstants.TTL) - 1);
				this.transmit(newMessage);
			}
			break;
		case SINK_LOCATION_CHANGE_BROADCAST_DYNAMIC:
			areaCenterAgent = (VCAgent) message
					.getValue(VCConstants.AREA_CENTER_AGENT);
			double distanceFromSinkToAreaCenter = 0;
			if(areaCenterAgent != null) {
				distanceToAreaCenter = Math.abs(DirectionalVCForwarding
						.getL2NormDistance(areaCenterAgent, this,
								areaCenterAgent, context));
				distanceFromSinkToAreaCenter = Math.abs(DirectionalVCForwarding
						.getL2NormDistance(areaCenterAgent,
								context.getSinkAgent(), areaCenterAgent,
								context));
			}
			if (updateVCTableOfNewSinkLocation(message)
					&& (((areaCenterAgent == null) && ((int) message
							.getValue(VCConstants.TTL) > 0)) || ((areaCenterAgent != null)
							&& (distanceToAreaCenter < context
									.getSinkMoveNotifyRadius()) && (distanceToAreaCenter < (distanceFromSinkToAreaCenter + 1))))) {
				ACLMessage newMessage = VCMessageHelper.createVCMessage(
						context, this, null, this.getVcLocalTable(),
						VCConstants.SinkMobilityMode.MOBILE);
				newMessage
						.setValue(
								constSensorNetwork.MODE,
								VCConstants.MessageMode.SINK_LOCATION_CHANGE_BROADCAST_DYNAMIC);
				newMessage.setValue(VCConstants.AREA_CENTER_AGENT,
						areaCenterAgent);
				newMessage.setValue(VCConstants.TTL,
						(int) message.getValue(VCConstants.TTL) - 1);
				this.transmit(newMessage);
			}
			break;
		case SINK_LOCATION_CHANGE_RANDOM_ROUTING:
			updateVCTableOfNewSinkLocation(message);
			if ((int) message.getValue(VCConstants.TTL) > 0) {
				VCAgent randomNeighbor = VCMessageHelper
						.getARandomNeighbor(this);
				ACLMessage newMessage = VCMessageHelper.createVCMessage(
						context, this, randomNeighbor, this.getVcLocalTable(),
						VCConstants.SinkMobilityMode.MOBILE);
				newMessage
						.setValue(
								constSensorNetwork.MODE,
								VCConstants.MessageMode.SINK_LOCATION_CHANGE_RANDOM_ROUTING);
				newMessage.setValue(VCConstants.TTL,
						(int) message.getValue(VCConstants.TTL) - 1);
				this.transmit(newMessage);
			}
			break;
		case SINK_LOCATION_CHANGE_CIRCLE:
			areaCenterAgent = (VCAgent) message.getValue(VCConstants.AREA_CENTER_AGENT);
			if (areaCenterAgent != null) {
				distanceToAreaCenter = Math.abs(DirectionalVCForwarding
						.getL2NormDistance(areaCenterAgent, this,
								areaCenterAgent, context));
			} else {
				distanceToAreaCenter = Math.abs(DirectionalVCForwarding
						.getL2NormDistance(this, this,
								MobileSinkHelper.lastBroadcastToEntireNetworkAgent, context));
			}
			// update the VC table. If there is no change, leave.
			vCTableUpdated = updateVCTableOfNewSinkLocation(message); 
			if (!vCTableUpdated) {
				break;
			}
			// broadcast to the entire network until TTL becomes 0
			if(areaCenterAgent == null && (int) message
							.getValue(VCConstants.TTL) > 0) {
				doBroadcast = true;
				if(isInLocalArea())
					setInPreviousLocalArea(true);
				else
					setInPreviousLocalArea(false);
				if(distanceToAreaCenter < context.getSinkMoveNotifyRadius() + 0
						&& distanceToAreaCenter > context.getSinkMoveNotifyRadius() - 4
						) {
					setInLocalArea(true);
				} else {
					setInLocalArea(false);
				}
			}
			// broadcast just inside the local area
			if(areaCenterAgent != null && distanceToAreaCenter < context.getSinkMoveNotifyRadius() + 0
					&& distanceToAreaCenter > context.getSinkMoveNotifyRadius() - 3
					) {
				doBroadcast = true;
				setInLocalArea(true);
			}
			if (doBroadcast) {
				ACLMessage newMessage = VCMessageHelper.createVCMessage(
						context, this, null, this.getVcLocalTable(),
						VCConstants.SinkMobilityMode.MOBILE);
				newMessage.setValue(constSensorNetwork.MODE,
						VCConstants.MessageMode.SINK_LOCATION_CHANGE_CIRCLE);
				newMessage.setValue(VCConstants.AREA_CENTER_AGENT,
						areaCenterAgent);
				newMessage.setValue(VCConstants.TTL,
						(int) message.getValue(VCConstants.TTL) - 1);
				this.transmit(newMessage);
			} 
			else if(distanceToAreaCenter < context.getSinkMoveNotifyRadius() && !message.getDestination().equals("*")) {
				@SuppressWarnings("unchecked")
				ArrayList<VCAgent> inPathAgents = (ArrayList<VCAgent>) message
						.getValue(VCConstants.IN_PATH_AGENTS);
				VCAgent nextRotateAgent = DirectionalVCForwarding.rotateAround(this,
						context.getSinkAgent(), inPathAgents, message, context);
				if (nextRotateAgent != null) {
					message.setValue(SENDER_AGENT, this.getNode());
					message.setDestination(nextRotateAgent.getNode().getName());
					this.setForwardingDestination(nextRotateAgent.getName());
					if (inPathAgents.size() >= 1) {
						inPathAgents.remove(0);
					}
					inPathAgents.add((VCAgent) message
							.getValue(VCConstants.PREVIOUS_AGENT));
					message.setValue(VCConstants.PREVIOUS_AGENT, this);
					context.getWorld().transmit(this.node, message);
				}
			}
//			updateVCTableOfNewSinkLocation(message);
//			if ((int) message.getValue(VCConstants.TTL) > 0) {
//				@SuppressWarnings("unchecked")
//				ArrayList<VCAgent> inPathAgents = (ArrayList<VCAgent>) message
//						.getValue(VCConstants.IN_PATH_AGENTS);
//				VCAgent nextRotateAgent = DirectionalVCForwarding.rotateAround(this,
//						context.getSinkAgent(), inPathAgents, message, context);
//				if (nextRotateAgent != null) {
//					message.setValue(SENDER_AGENT, this.getNode());
//					message.setDestination(nextRotateAgent.getNode().getName());
//					this.setForwardingDestination(nextRotateAgent.getName());
//					if (inPathAgents.size() >= 1) {
//						inPathAgents.remove(0);
//					}
//					inPathAgents.add((VCAgent) message
//							.getValue(VCConstants.PREVIOUS_AGENT));
//					message.setValue(VCConstants.PREVIOUS_AGENT, this);
//					context.getWorld().transmit(this.node, message);
//				}
//			}
			break;
		case DIRECTIONAL_VC:
			if ((message.getValue(SINK_AGENT)).equals(context)) {
				DirectionalVCForwarding.finishRoutig(context, true, message);
				return;
			}
			message.setValue(VCConstants.TTL,
					(int) message.getValue(VCConstants.TTL) - 1);
			if ((int) message.getValue(VCConstants.TTL) < 0) {
				DirectionalVCForwarding.finishRoutig(context, false, message);
				// TextUi.print("Routing-Failed: " +
				// context.getPathRecord().toString());
				return;
			}
			if (message.getValue(VCConstants.MOBILITY_MODE).equals(
					VCConstants.SinkMobilityMode.STATIC)) {
				VCAgent destination = (VCAgent) message.getValue(SINK_AGENT);
				nextAgentToSink = DirectionalVCForwarding.getNextHopUsingDVCR(this,
						destination,
						(VCAgent) message.getValue(VCConstants.PREVIOUS_AGENT),
						context);
			} else {
				updateVCTableOfNewSinkLocation(message);
				
				if (nextAgentToSink.equals(context.getSinkAgent())) {
					DirectionalVCForwarding
							.finishRoutig(context, true, message);
					return;
				}
			}
			if (nextAgentToSink == null) {
				// TextUi.print("Routing-Failed: " +
				// context.getPathRecord().toString());
				DirectionalVCForwarding.finishRoutig(context, false, message);
				return;
			}
			// else if(((SensorNode)
			// message.getValue(SINK_NODE)).getName().equals(nextAgent.getNode().getName())){
			// // TextUi.println("Routing-Successful: " +
			// context.getPathRecord().toString());
			// DirectionalVCForwarding.successfulRoutings++;
			// }
			message.setDestination(nextAgentToSink.getName());
			this.setForwardingDestination(nextAgentToSink.getName());
			message.setValue(VCConstants.PREVIOUS_AGENT, this);
			context.getWorld().transmit(this.node, message);
			// context.getPathRecord().add(this.getName());
			// TextUi.println(context.getPathRecord().toString());
			break;
		case RUMOR_ROUTING:
			message.setValue(VCConstants.TTL,
					(int) message.getValue(VCConstants.TTL) - 1);
			// If TTL = 0 then don't update the VC nodes, i.e., packet is
			// dropped
			if (updateVirtualCoordinates(message) == 0) {
				context.getWorld().transmit(this.node, message);
				// if anchor node is in vc mode, then sends its VC table to all
				// other anchors
				// if(isAnchor &&
				// VCMessageHelper.getNumberOfNodesInVCMode(context) ==
				// context.getSensorNodeCount()) {
				// List<VCAgent> anchorAgents = VCContext.getAnchorAgents(
				// context.getWorld(), ForwarderSensorAgent.class);
				// for (VCAgent anchor : anchorAgents) {
				// ACLMessage newMessage =
				// VCMessageHelper.createVCMessage(context, getName(),
				// anchor.getName(), getVcLocalTable());
				// DirectionalVCForwarding.sendUsingDVCR(this, anchor,
				// newMessage, context);
				// }
				// }
			}
			break;
		default:
			break;
		}
	}

	public boolean updateVCTableOfNewSinkLocation(ACLMessage message) {
		boolean updated = false;
		List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
				context.getWorld(), ForwarderSensorAgent.class);
		if (((VCLocalTable) message.getValue(VCConstants.VC_LOCAL_TABLE))
				.getUpdateTime() > this.getVcLocalTable().getUpdateTime()) {
			for (VCAgent anchor : anchorAgents) {
				this.getVcLocalTable()
						.setNumberOfHops(
								context.getSinkAgent(),
								anchor,
								((VCLocalTable) message
										.getValue(VCConstants.VC_LOCAL_TABLE))
										.getNumberOfHops(
												context.getSinkAgent(), anchor));
			}
			this.getVcLocalTable().setUpdateTime(
					((VCLocalTable) message
							.getValue(VCConstants.VC_LOCAL_TABLE))
							.getUpdateTime());
			updated = true;
		} else if (((VCLocalTable) message.getValue(VCConstants.VC_LOCAL_TABLE))
				.getUpdateTime() < this.getVcLocalTable().getUpdateTime()) {
			for (VCAgent anchor : anchorAgents) {
				((VCLocalTable) message
						.getValue(VCConstants.VC_LOCAL_TABLE))
						.setNumberOfHops(
								context.getSinkAgent(),
								anchor,
								this.getVcLocalTable().getNumberOfHops(
										context.getSinkAgent(), anchor));
			}
			((VCLocalTable) message.getValue(VCConstants.VC_LOCAL_TABLE))
					.setUpdateTime(this.getVcLocalTable().getUpdateTime());
			updated = true;
		}
		if(updated || nextAgentToSink == null) {
			VCAgent destination;
			if((VCAgent) message.getValue(SINK_AGENT) == null)
				destination = context.getSinkAgent();
			else
				destination = (VCAgent) message.getValue(SINK_AGENT);
			nextAgentToSink = DirectionalVCForwarding.getNextHopUsingDVCR(this,
					destination,
					(VCAgent) message.getValue(VCConstants.PREVIOUS_AGENT),
					context);
		}
		return updated;
	}

	@Override
	public int hashCode() {
		// TODO Auto-generated method stub
		return super.hashCode();
	}

	public boolean isAnchor() {
		return isAnchor;
	}

	public boolean isInLocalArea() {
		return isInLocalArea;
	}

	public boolean isInVCMode() {
		return isInVCMode;
	}

	public void setAnchor(boolean isAnchor) {
		this.isAnchor = isAnchor;
	}

	public void setContext(VCContext context) {
		this.context = context;
	}

	public void setCorrectVCLocalTable(VCLocalTable correctVCLocalTable) {
		this.correctVCLocalTable = correctVCLocalTable;
	}

	public void setInLocalArea(boolean isInLocalArea) {
		this.isInLocalArea = isInLocalArea;
	}

	public void setInVCMode(boolean isInVCMode) {
		this.isInVCMode = isInVCMode;
	}

	public void setNeighbors(ArrayList<VCAgent> neighbors) {
		this.neighbors = neighbors;
	}

	public void setVcLocalTable(VCLocalTable vcLocalTable) {
		this.vcLocalTable = vcLocalTable;
	}

	/**
	 * Visualization for the virtual coordinate agent
	 */
	@Override
	public String toString() {
		Formatter fmt = new Formatter();
		fmt.add("VirtualCoordinateAgent");
		fmt.indent();
		fmt.add(toStringCommonProperties());
		fmt.add(toStringForwardingDestination());
		return fmt.toString();

	}

	@Override
	protected String toStringCommonProperties() {
		Formatter fmt = new Formatter();
		fmt.is("Name", getName());
		fmt.is("Location", getNode().getLocation());
		fmt.is("Transmission range", getTransmissionRange());
		fmt.is("Sensor range", getSensorRange());
		fmt.is("Is Anchor", isAnchor ? "True" : "False");
		return fmt.toString();

	}

	/**
	 * updates the number of hops to anchors of the message or the local data in
	 * agent
	 * 
	 * @param message
	 */
	public int updateVirtualCoordinates(ACLMessage message) {

		messagesForwardedWithoutVCUpdate++;

		// TextUi.println(message.getValue(VCConstants.TTL));
		if ((int) message.getValue(VCConstants.TTL) < 0) {
			context.setMessagesInNetworkCount(context
					.getMessagesInNetworkCount() - 1);
			return 1;
		}
		// TextUi.println(((VCLocalTable)message.getValue(VCConstants.VC_LOCAL_TABLE)).hopsFromNodesToAnchors.size());
		VCAgent randomDestination = VCMessageHelper.getARandomNeighbor(this);
		setForwardingDestination(randomDestination.getName());
		message.setDestination(randomDestination.getName());
		VCAgent sender = VCMessageHelper.getVCAgentByName(context.getWorld(),
				message.getSender());
		this.setVcLocalTable(VCLocalTable.combineVCLocalTables(sender, this,
				this.getVcLocalTable(),
				(VCLocalTable) message.getValue(VCConstants.VC_LOCAL_TABLE)));
		// updates the VC Table information in the packet's message
		message.setValue(VCConstants.VC_LOCAL_TABLE, this.vcLocalTable);
		for (AbstractSensorAgent neighbor : neighbors) {
			VCAgent vcNeighbor = (VCAgent) neighbor;
			vcNeighbor.setVcLocalTable(VCLocalTable.combineVCLocalTables(this,
					vcNeighbor, vcNeighbor.getVcLocalTable(),
					this.getVcLocalTable()));
		}
		if (!context.isForwardVCofAllNodes() || isAnchor) {
			vcLocalTable.setNumberOfHops(this, this, 0);
		}
		return 0;
	}

	public boolean isInPreviousLocalArea() {
		return isInPreviousLocalArea;
	}

	public void setInPreviousLocalArea(boolean isInPreviousLocalArea) {
		this.isInPreviousLocalArea = isInPreviousLocalArea;
	}
	

	public VCAgent(String name, VCContext context) {
		super(name, context.getWorld());
		this.context = context;
	}

	@Override
	protected void afterProcessingPerceptions() {

	}

	@Override
	protected void beforeProcessingPerceptions() {
		super.beforeProcessingPerceptions();
	}

	@Override
	public boolean equals(Object arg0) {
		if (arg0 instanceof VCAgent) {
			return this.getName().equals(((VCAgent) arg0).getName());
		} else {
			return super.equals(arg0);
		}
	}

	public VCContext getContext() {
		return context;
	}

	public VCLocalTable getCorrectVCLocalTable() {
		return correctVCLocalTable;
	}

	public int getMessagesForwardedWithoutVCUpdate() {
		return messagesForwardedWithoutVCUpdate;
	}

	public ArrayList<VCAgent> getNeighbors() {
		return neighbors;
	}

	public VCLocalTable getVcLocalTable() {
		return vcLocalTable;
	}

	@Override
	protected void handleIntruderPresence(Perception p) {
		// TODO Not implemented yet

	}


}
