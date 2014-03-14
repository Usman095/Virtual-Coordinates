package agents;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import yaes.framework.agent.ACLMessage;
import yaes.sensornetwork.constSensorNetwork;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.model.SensorNetworkMessageConstants;
import yaes.virtualcoordinate.VCConstants;
import yaes.virtualcoordinate.VCContext;
import yaes.virtualcoordinate.VCSimulation;
/**
 * 
 * This class will do directional routing using the algorithm in the paper: 
 * Directional Virtual Coordinate System for Wireless Sensor Networks
 *
 */
public class DirectionalVCForwarding implements Serializable, constSensorNetwork, SensorNetworkMessageConstants, VCConstants {

	private static final long serialVersionUID = 3725968166916518394L;
	
	public static boolean useOnlyLocalVCInformation = true;
	public static int solveEquationFails = 0;

	public static void sendUsingDVCR(VCAgent sourceAgent, VCAgent destinationAgent, ACLMessage message, VCContext context) {
//		TextUi.println("Source: " + sourceAgent.getName() + " Destination: " + destinationAgent.getName());
		message.setValue(MODE, VCConstants.MessageMode.DIRECTIONAL_VC);
		VCAgent nextAgent = DirectionalVCForwarding.getNextHopUsingDVCR(sourceAgent, destinationAgent, 
				(VCAgent) message.getValue(VCConstants.PREVIOUS_AGENT), context);
		if(nextAgent == null) {
			finishRoutig(context, false, message);
    		return;
		}
		if(nextAgent.getName().equals(destinationAgent.getName())){
			finishRoutig(context, true, message);
    		return;
    	}
		message.setDestination(nextAgent.getName());	
		sourceAgent.setForwardingDestination(nextAgent.getName());
		sourceAgent.transmit(message);
//		VCContext.getPathRecord().add(nextHop.getName());
//		TextUi.println(VCContext.getPathRecord().toString());
	}
	
	/**
	 * This function will try to find the next node in a path which rotates arount the sink.
	 * @param current
	 * @param message
	 * @param context
	 * @return
	 */
	public static VCAgent rotateAround(VCAgent current, VCAgent destination, ArrayList<VCAgent> inPathAgents, ACLMessage message, VCContext context){
		VCAgent prevAgent = (VCAgent) message.getValue(VCConstants.PREVIOUS_AGENT);
		
		VCAgent mostUpdatedAgent = current;
		for(VCAgent neighbor : current.getNeighbors()){
			if(neighbor.getVcLocalTable().getUpdateTime() > mostUpdatedAgent.getVcLocalTable().getUpdateTime())
				mostUpdatedAgent = neighbor;
		}
		VCAgent minHopAgent = null;
		double minDistanceDifference = Double.valueOf(Double.POSITIVE_INFINITY );
		double maxDistanceToPrev = Double.valueOf(Double.NEGATIVE_INFINITY );
		for(VCAgent neighbor : current.getNeighbors()){
			if(neighbor.getName().equals(prevAgent.getName()))
				continue;
//			if(prevAgent.getNeighbors().contains(neighbor))
//				continue;
			double distanceDifferenceToDestination = Math.abs(Math.abs(getL2NormDistance(mostUpdatedAgent, neighbor, destination, context)) - context.getSinkMoveNotifyRadius()-3);
//			TextUi.println("distanceToDestination: " + getL2NormDistance(mostUpdatedAgent, neighbor, destination, context));
			double distanceToPrev = Math.abs(getL2NormDistance(prevAgent, neighbor, prevAgent, context));
			for(VCAgent inPathAgent : inPathAgents) {
				distanceToPrev += Math.abs(getL2NormDistance(inPathAgent, neighbor, inPathAgent, context));
			}
			double bestDistance = distanceDifferenceToDestination - distanceToPrev/Math.min((inPathAgents.size()+1), 3);
//			TextUi.println("distanceToPrev: " + distanceToPrev);
//			if(distanceToPrev > maxDistanceToPrev) {
				if(bestDistance < minDistanceDifference) {
					minDistanceDifference = bestDistance;
					maxDistanceToPrev = distanceToPrev;
					minHopAgent = neighbor;
				}				
//			}
		}
		if(minHopAgent == null ||  minHopAgent.getVcLocalTable().getUpdateTime() >= current.getVcLocalTable().getUpdateTime())
			return null;
		return minHopAgent;
//		return null;
	}
	
	/**
	 * This function will do the directional virtual coordinate routing.
	 * It will one of the neighbors of current node as the next hop to reach destination
	 * @param current
	 * @param message
	 * @param context
	 * @return
	 */
	public static VCAgent getNextHopUsingDVCR(VCAgent current, VCAgent destination, VCAgent prevAgent, VCContext context){
		HashMap<VCAgent, Double> neighborDistances = new HashMap<>();
		
		VCAgent mostUpdatedAgent = current;
		for(VCAgent neighbor : current.getNeighbors()){
			if(neighbor.getVcLocalTable().getUpdateTime() > mostUpdatedAgent.getVcLocalTable().getUpdateTime())
				mostUpdatedAgent = neighbor;
		}
		double minDistance = Double.valueOf(Double.POSITIVE_INFINITY );
		double maxDistance = Double.valueOf(Double.NEGATIVE_INFINITY );
		for(VCAgent neighbor : current.getNeighbors()){
			if(neighbor.equals(destination))
				return neighbor;
			if(neighbor.equals(prevAgent))
				continue;
			double distToDestination = getL2NormDistance(mostUpdatedAgent, neighbor, destination, context);
			neighborDistances.put(neighbor, distToDestination);
			if(distToDestination < minDistance) {
				minDistance = distToDestination;
			} 
			if(distToDestination > maxDistance) {
				maxDistance = distToDestination;
			}
		}
		double currentDistance = getL2NormDistance(mostUpdatedAgent, current, destination, context);
		if(minDistance <= maxDistance) {
			if(minDistance !=0 && ( minDistance < currentDistance)) { // next hop found
				ArrayList<VCAgent> minNeighborDistances = new ArrayList<>();
				for(VCAgent neighbor: neighborDistances.keySet())
					if(neighborDistances.get(neighbor) == minDistance)
						minNeighborDistances.add(neighbor);
				return minNeighborDistances.get(context.getRandom().nextInt(minNeighborDistances.size()));
			}
		} 
		if(current.getNeighbors().size() == 0)
			return null;
		VCAgent minHopAgent = current.getNeighbors().get(0);
		List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
				context.getWorld(), ForwarderSensorAgent.class);
//		List<List<VCAgent>> powersetofAnchors = powerSet(anchorAgents);
//		powersetofAnchors.toArray();
//		for(int i = powersetofAnchors.size()-1; i >= 0; i--) {
//			if(powersetofAnchors.get(i).size() != 4)
//				continue;
			minDistance = Double.valueOf(Double.POSITIVE_INFINITY );
			try {
				for(VCAgent neighbor : current.getNeighbors()){
					if(neighbor.equals(prevAgent))
						continue;
//					double distToDestination = getApproximatedHopsToDest(
//							neighbor, destination, powersetofAnchors.get(i), context);
					double distToDestination = getApproximatedHopsToDest(
							neighbor, destination, anchorAgents.subList(0, 4), context);
					if(distToDestination < minDistance) {
						minDistance = distToDestination;
						minHopAgent = neighbor;
					}
				}
				return minHopAgent;
//				currentDistance = getApproximatedHopsToDest(
//						current, destination, powersetofAnchors.get(i), context);
			} catch (RuntimeException e) {
				solveEquationFails++;
//				continue;
			}
			
//		}
		return null;
	}
	/**
	 * This method is calculating the L2 distance between two agents by calculating the 
	 * transformed coordinates as stated in DVCR paper. It seems that Nid in the formula
	 * in that paper should be changed to Ndj
	 * @param agent1
	 * @param agent2
	 * @param context
	 * @return
	 */
	public static double getL2NormDistance(VCAgent mostUpdatedNeighbor,VCAgent agent1, VCAgent agent2, VCContext context) {
		List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
				context.getWorld(), ForwarderSensorAgent.class);
		double l2norm = 0;
		for(int i = 0; i < anchorAgents.size(); i++) {
			for(int j = i+1; j < anchorAgents.size(); j++) {
				VCAgent anchor1 = anchorAgents.get(i);
				VCAgent anchor2 = anchorAgents.get(j);
				double Nij = 1.0/(2.0*getVCTable(agent1, false).getNumberOfHops(anchor1, anchor2)) *
						( Math.pow(getVCTable(agent1, false).getNumberOfHops(agent1, anchor1), 2) - 
								Math.pow(getVCTable(agent1, false).getNumberOfHops(agent1, anchor2), 2));
				double Ndj = 1.0/(2.0*getVCTable(agent1, false).getNumberOfHops(anchor1, anchor2)) *
						( Math.pow(getVCTable(mostUpdatedNeighbor, agent2.equals(context.getSinkAgent())).getNumberOfHops(agent2, anchor1), 2) - 
								Math.pow(getVCTable(mostUpdatedNeighbor, agent2.equals(context.getSinkAgent())).getNumberOfHops(agent2, anchor2), 2));
				l2norm += Math.pow(Nij - Ndj, 2);
				
			}
		}
		l2norm = Math.sqrt(l2norm);
		return l2norm;
	}
	/**
	 * This methods calculates the hop-count from the current node
	 * to the destination.
	 * The variables used in the method such as alpha12 etc are taken
	 * from the paper "Directional Virtual Coordinates Systems for 
	 * Wireless Sensor Networks"
	 * @param current
	 * @param dest
	 * @param context
	 * @return
	 */
	public static double getApproximatedHopsToDest(VCAgent current, VCAgent dest, List<VCAgent> anchorAgents, VCContext context) throws RuntimeException{
		ArrayList<Double> neighborDistances = new ArrayList<Double>();
		for(VCAgent neighbor : current.getNeighbors()) {
				neighborDistances.add(virtualDistance(current,  neighbor, 
						anchorAgents.get(0), anchorAgents.get(1), false));
		}
		double alpha12 = Collections.min(neighborDistances);
		double beta12 = Collections.max(neighborDistances);
		
		neighborDistances.clear();
		for(VCAgent neighbor : current.getNeighbors()) {
			neighborDistances.add(virtualDistance(current,  neighbor, 
					anchorAgents.get(2), anchorAgents.get(3), false));
		}
		double alpha34 = Collections.max(neighborDistances);
		double beta34 = Collections.min(neighborDistances);
		
		double distToDest12 = virtualDistance(current, dest, 
				anchorAgents.get(0), anchorAgents.get(1), current.equals(context.getSinkAgent()) && dest.equals(context.getSinkAgent()));
		double distToDest34 = virtualDistance(current, dest, 
				anchorAgents.get(2), anchorAgents.get(3), current.equals(context.getSinkAgent()) && dest.equals(context.getSinkAgent()));
		
		
		// jscience package was used to solve the equation better. 
		// It probably uses psuedo inverse when matrix is singular. 
//		Float64Matrix A = Float64Matrix.valueOf(new double[][] {{alpha12, beta12},{alpha34, beta34}});
//		Float64Matrix B = Float64Matrix.valueOf(new double[][]{{distToDest12},{distToDest34}});
//		org.jscience.mathematics.vector.Matrix<Float64> x = A.solve(B);
//		double n = x.get(0, 0).doubleValue();
//		double m = x.get(1, 0).doubleValue();
		
		
		double n = (distToDest12*beta34 - distToDest34*beta12)/(alpha12*beta34-alpha34*beta12);
		double m = -(distToDest12*alpha34 - distToDest34*alpha12)/(alpha12*beta34-alpha34*beta12);
		
		return Math.abs(n)+Math.abs(m);
	}

	/**
	 * Virtual direction: Each node acts as a unit vector in the direction of A1 -> A2
	 * where A1, A2 are reference anchors 
	 * @param agent
	 * @param anchor1
	 * @param anchor2
	 * @return
	 */
	public static double unitVector(VCAgent agent,VCAgent anchor1, VCAgent anchor2, boolean useLocal){
		double hNiAj = getVCTable(agent, useLocal).getNumberOfHops(agent, anchor1);
		double hNiAk = getVCTable(agent, useLocal).getNumberOfHops(agent, anchor2);
		double hAjAk = getVCTable(agent, useLocal).getNumberOfHops(anchor1, anchor2);
		return 1/(2* hAjAk) * (Math.pow(hNiAj, 2) - Math.pow(hNiAk, 2));
	}
	
	/**
	 * This method returns the virtual distance between two nodes
	 * @param agent1
	 * @param agent2
	 * @param anchor1
	 * @param anchor2
	 * @return
	 */
	public static double virtualDistance(VCAgent agent1, VCAgent agent2,
			VCAgent anchor1, VCAgent anchor2, boolean useLocal){
		return Math.abs(unitVector(agent1, anchor1, anchor2, useLocal) - 
				unitVector(agent2, anchor1, anchor2, useLocal));
	}
	
	public static <T> List<List<T>> powerSet(List<T> originalList) {
        List<List<T>> sets = new ArrayList<List<T>>();
        if (originalList.isEmpty()) {
            sets.add(new ArrayList<T>());
            return sets;
        }
        List<T> list = new ArrayList<T>(originalList);
        T head = list.get(0);
        List<T> rest = new ArrayList<T>(list.subList(1, list.size()));
        for (List<T> set : powerSet(rest)) {
            List<T> newList = new ArrayList<T>();
            newList.add(head);
            newList.addAll(set);
            sets.add(newList);
            sets.add(set);
        }
        return sets;
    }
	
	private static VCLocalTable getVCTable(VCAgent agent, boolean useLocal) {
		if(useOnlyLocalVCInformation && useLocal) {
			return agent.getVcLocalTable();
		}
		else {
			return agent.getContext().getCorrectVCLocalTable();
		}
	}
	
	public static void finishRoutig(VCContext context, boolean successful, ACLMessage message) {
		
		if(successful) {
			context.getSimulationOutput().update(VCConstants.SuccessfullRoutings, 1);
			context.getSimulationOutput().update(VCConstants.RoutedMessages, (int)message.getValue(VCConstants.PATH_LENGTH));
		}
		else
			context.getSimulationOutput().update(VCConstants.FailedRoutings, 1);
		for(int i = 0; i < VCSimulation.sources.size(); i++) {
			if(((VCAgent)message.getValue(VCConstants.SINK_AGENT)).equals(VCSimulation.destinations.get(i)) 
					&& ((VCAgent)message.getValue(VCConstants.SENDER_AGENT)).getName().equals(VCSimulation.sources.get(i).getName())) {
				VCSimulation.sources.remove(i);
				VCSimulation.destinations.remove(i);
			}
		}
	}
}
