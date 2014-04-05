package agents;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import yaes.framework.agent.ACLMessage;
import yaes.sensornetwork.agents.AbstractSensorAgent;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.agents.SensorRoutingHelper;
import yaes.sensornetwork.model.SensorNetworkMessageConstants;
import yaes.sensornetwork.model.SensorNetworkWorld;
import yaes.sensornetwork.model.SensorNode;
import yaes.ui.text.TextUi;
import yaes.virtualcoordinate.VCConstants;
import yaes.virtualcoordinate.VCConstants.NetworkMode;
import yaes.virtualcoordinate.VCContext;

/**
 * 
 * @author Rouhollah
 * 
 */

public class VCMessageHelper implements Serializable, VCConstants {

	private static final long serialVersionUID = -7386832953750300459L;

	/**
	 * Creates a message which contains virtual coordinates to each anchor
	 * 
	 * @return
	 */
	public static ACLMessage createVCMessage(VCContext context, VCAgent sender,
			VCAgent destination, VCLocalTable vcLocalTable,
			VCConstants.SinkMobilityMode mobilityMode) {
		final ACLMessage message = new ACLMessage(sender.getName(),
				ACLMessage.Performative.INFORM);
		if (destination == null)
			message.setDestination("*");
		else
			message.setDestination(destination.getName());
		message.setValue(SensorNetworkMessageConstants.FIELD_CONTENT,
				SensorNetworkMessageConstants.MESSAGE_DATA);
		message.setValue(SensorNetworkMessageConstants.FIELD_INTENSITY, 0);
		message.setValue(VCConstants.PATH_LENGTH, 0);
		message.setValue(VCConstants.PREVIOUS_AGENT, sender);
		message.setValue(VCConstants.IN_PATH_AGENTS, new ArrayList<VCAgent>());
		message.setValue(VCConstants.SENDER_AGENT, sender);
		message.setValue(VCConstants.SINK_AGENT, destination);
		message.setValue(VCConstants.TTL, context.getTTLCount());
		message.setValue(VCConstants.VC_LOCAL_TABLE, vcLocalTable);
		message.setValue(VCConstants.MOBILITY_MODE, mobilityMode);
		context.getSimulationOutput().update(VCConstants.MessagesGenerated, 1);
		// message.setValue(VCConstants.FIELD_MODE,
		// context.getSimulationInput().getParameterEnum(NetworkMode.class));
		return message;
	}

	/**
	 * This method returns the accumulated average error of the whole network
	 * 
	 * @param context
	 * @return
	 */
	public static double getAverageError(VCContext context) {
		List<VCAgent> allAgents = VCMessageHelper.getAllVCAgents(
				context.getWorld(), false);
		double error = 0;
		for (VCAgent agent : allAgents) {
			// The averaged error from source to VC anchors (anchors are also
			// VCAgents)
			Set<MySimpleEntry<VCAgent, VCAgent>> keyArray = agent
					.getVcLocalTable().keySet();
			for (MySimpleEntry<VCAgent, VCAgent> key : keyArray) {
				if (agent.getVcLocalTable().containsKey(key)) {
					error += Math.abs(context.getCorrectVCLocalTable()
							.getNumberOfHops(key)
							- agent.getVcLocalTable().getNumberOfHops(key));
				}
			}
			// error +=
			// (context.getCorrectVCLocalTable().hopsFromNodesToAnchors.size() -
			// agent.getVcLocalTable().hopsFromNodesToAnchors.size());
		}
		return (double) error / allAgents.size() / allAgents.size()
				/ context.getAnchorCount();
	}

	/**
	 * This methods gives the summation of the number of messages that pass
	 * through the network without updating the VC at nodes
	 * 
	 * @param context
	 * @return
	 */
	public static double getAverageForwardedMessagesWithoutVCUpdate(
			VCContext context) {
		List<VCAgent> allAgents = VCMessageHelper.getAllVCAgents(
				context.getWorld(), false);
		int sum = 0;
		for (VCAgent agent : allAgents) {
			sum += agent.getMessagesForwardedWithoutVCUpdate();
		}
		return (double) sum / allAgents.size();
	}

	/**
	 * This method returns the number of the nodes which have fully updated VC
	 * table. Note that all the nodes should have the final VC Table
	 * synchronized with the sink. The sink has the information to assess the
	 * validity of the VC information at each node
	 * 
	 * @param context
	 * @return
	 */
	public static double getNumberOfNodesInVCMode(VCContext context) {
		List<VCAgent> allAgents = VCMessageHelper.getAllVCAgents(
				context.getWorld(), false);
		int numberOfNodesInVCMode = 0;
		for (VCAgent agent : allAgents) {
			if (!context.isForwardVCofAllNodes() && !agent.isAnchor())
				continue;
			Set<MySimpleEntry<VCAgent, VCAgent>> keyArray = agent
					.getVcLocalTable().keySet();
			boolean allchecked = true;
			for (MySimpleEntry<VCAgent, VCAgent> key : keyArray) {
				if ((!agent.getVcLocalTable().containsKey(key) || context
						.getCorrectVCLocalTable().getNumberOfHops(key) != agent
						.getVcLocalTable().getNumberOfHops(key))
						&& (context.isForwardVCofAllNodes() || key.getKey()
								.isAnchor())) {
					allchecked = false;
					break;
				}
			}
			// TextUi.println("KeyarraySize: " +
			// vcAgent.getVcLocalTable().hopsFromNodesToAnchors.keySet().size()
			// + "   " + context.getAnchorCount()*(context.getAnchorCount()-1));

			if (context.isForwardVCofAllNodes()) {
				if (allchecked
						&& keyArray.size() == context.getCorrectVCLocalTable()
								.keySet().size()) {
					numberOfNodesInVCMode++;
					agent.setInVCMode(true);
				}
			} else {
				if (!agent.isAnchor())
					continue;
				List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
						context.getWorld(), ForwarderSensorAgent.class);
				allchecked = true;
				for (VCAgent anchor1 : anchorAgents) {
					for (VCAgent anchor2 : anchorAgents) {
						if (!agent.getVcLocalTable().containsKey(
								new MySimpleEntry<VCAgent, VCAgent>(anchor1,
										anchor2))) {
							allchecked = false;
							break;
						}
					}
				}
				if (allchecked) {
					numberOfNodesInVCMode++;
					agent.setInVCMode(true);
				}
			}
		}
		return numberOfNodesInVCMode;
	}

	public static VCAgent getVCAgentByName(SensorNetworkWorld world, String name) {
		List<VCAgent> agents = VCMessageHelper.getAllVCAgents(world, true);

		for (int i = 0; i < agents.size(); i++) {
			if ((agents.get(i)).getName().equals(name))
				return agents.get(i);
		}
		return null;
	}

	public static List<VCAgent> getAllVCAgents(SensorNetworkWorld world,
			boolean includeSink) {
		List<AbstractSensorAgent> allAgents = SensorRoutingHelper
				.getSensorAgents(world, AbstractSensorAgent.class);
		if (includeSink)
			allAgents.add(world.getSinkNode().getAgent());
		@SuppressWarnings("unchecked")
		List<VCAgent> allVCAgents = (List<VCAgent>) (List<?>) allAgents;
		return allVCAgents;
	}

	public static List<VCAgent> getAnchorAgents(SensorNetworkWorld world,
			Class<ForwarderSensorAgent> agentClass) {
		List<VCAgent> agents = getAllVCAgents(world, true);
		// TextUi.println("Agents count: " + agents.size());
		List<VCAgent> anchorAgents = new ArrayList<VCAgent>();
		for (int i = 0; i < agents.size(); i++) {
			if ((agents.get(i)).isAnchor())
				anchorAgents.add((VCAgent) agents.get(i));
		}
		// TextUi.println("Anchors count: " + anchorAgents.size());
		return anchorAgents;
	}

	/**
	 * This information is used for comparison analysis
	 * 
	 * @param context
	 */
	public static void setShortestPathToAnchorsOldVersion(VCContext context) {
		SensorNetworkWorld world = context.getWorld();
		List<VCAgent> allAgents = getAllVCAgents(context.getWorld(), true);
		List<VCAgent> sourceAgents = new ArrayList<>(allAgents);
		List<VCAgent> anchorAgents = getAnchorAgents(world,
				ForwarderSensorAgent.class);
		int count = 0;
		for (VCAgent source : sourceAgents) {
			for (VCAgent anchor : anchorAgents) {
				if (source.getName().equals(anchor.getName())) {
					context.getCorrectVCLocalTable().setNumberOfHops(source,
							anchor, 0);

					continue;
				}
				@SuppressWarnings("unchecked")
				List<String> path = SensorRoutingHelper.getShortestPath(
						(List<AbstractSensorAgent>) (List<?>) allAgents,
						source, anchor);
				if (path.size() < 2) {
					TextUi.println("Path finding failed for "
							+ source.getName() + " to " + anchor.getName());
//					TextUi.println("Returned path was: " + path);
//					TextUi.println("This most likely means that the graph is not connected.");
					// System.exit(1);
				}
				context.getCorrectVCLocalTable().setNumberOfHops(source,
						anchor, path.size() - 1);
				count++;
				TextUi.println("Progress: " + count + " / "
						+ sourceAgents.size() * anchorAgents.size()
						+ " Percentage: " + count
						/ (sourceAgents.size() * anchorAgents.size()) * 100);
			}
		}
		TextUi.println("Copying the coordinates...");
		// copy correct VC tables to all agents
		VCAgent sinkAgent = context.getSinkAgent();
		for (VCAgent agent : allAgents) {
			for (VCAgent anchor1 : anchorAgents) {
				// for(VCAgent anchor2 : anchorAgents) {
				// agent.getVcLocalTable().setNumberOfHops(anchor1, anchor2,
				// context.getCorrectVCLocalTable().getNumberOfHops(anchor1,
				// anchor2));
				// }
				agent.getVcLocalTable().setNumberOfHops(
						sinkAgent,
						anchor1,
						context.getCorrectVCLocalTable().getNumberOfHops(
								sinkAgent, anchor1));
				// agent.getVcLocalTable().setNumberOfHops(agent, anchor1,
				// context.getCorrectVCLocalTable().getNumberOfHops(agent,
				// anchor1));
			}

		}
	}

	/**
	 * This information is used for comparison analysis
	 * 
	 * @param context
	 */
	public static void setShortestPathToAnchors(VCContext context) {
		SensorNetworkWorld world = context.getWorld();
		List<VCAgent> allAgents = getAllVCAgents(context.getWorld(), true);
		List<VCAgent> sourceAgents = new ArrayList<>(allAgents);
		List<VCAgent> anchorAgents = getAnchorAgents(world,
				ForwarderSensorAgent.class);
		int count = 0;
		for (VCAgent anchor : anchorAgents) {
			//
			//  Calculate the path from all the anchors to one of them in one shot. 
			//  FIXME: this returns the reverse paths, so one need to reverse them.
			//
			@SuppressWarnings("unchecked")			
			List<List<String>> paths = SensorRoutingHelper.getShortestPaths(
					(List<AbstractSensorAgent>) (List<?>) allAgents, anchor,
					(List<AbstractSensorAgent>) (List<?>) sourceAgents);
			for(int i = 0; i!=sourceAgents.size(); i++) {
				VCAgent source = sourceAgents.get(i);
				if (source.getName().equals(anchor.getName())) {
					context.getCorrectVCLocalTable().setNumberOfHops(source,
							anchor, 0);
					continue;
				}
				List<String> path = new ArrayList<>(); 
				path.addAll(paths.get(i));
				Collections.reverse(path);				
				TextUi.println("Path from " + source.getName() + " to " + anchor.getName());
				TextUi.println(path);
				if (path.size() < 2) {
					TextUi.println("Path finding failed for "
							+ source.getName() + " to " + anchor.getName());
					TextUi.println("Returned path was: " + path);
					TextUi.println("This most likely means that the graph is not connected.");
					// System.exit(1);
				}
				context.getCorrectVCLocalTable().setNumberOfHops(source,
						anchor, path.size() - 1);
				count++;
				TextUi.println("Progress: " + count + " / "
						+ sourceAgents.size() * anchorAgents.size()
						+ " Percentage: " + count
						/ (sourceAgents.size() * anchorAgents.size()) * 100);
			}
		}
		TextUi.println("Copying the coordinates...");
		// copy correct VC tables to all agents
		VCAgent sinkAgent = context.getSinkAgent();
		for (VCAgent agent : allAgents) {
			for (VCAgent anchor1 : anchorAgents) {
				 for(VCAgent anchor2 : anchorAgents) {
				 agent.getVcLocalTable().setNumberOfHops(anchor1, anchor2,
				 context.getCorrectVCLocalTable().getNumberOfHops(anchor1,
				 anchor2));
				 }
				agent.getVcLocalTable().setNumberOfHops(
						sinkAgent,
						anchor1,
						context.getCorrectVCLocalTable().getNumberOfHops(
								sinkAgent, anchor1));
				 agent.getVcLocalTable().setNumberOfHops(agent, anchor1,
				 context.getCorrectVCLocalTable().getNumberOfHops(agent,
				 anchor1));
			}

		}
	}

	public static VCAgent getARandomNeighbor(VCAgent agent) {
		ArrayList<VCAgent> neighbors = agent.getNeighbors();
		int randomIndex = (int) (agent.getContext().getRandom()
				.nextInt(neighbors.size()));
		return neighbors.get(randomIndex);
	}
}
