package agents;

import java.io.Serializable;

import yaes.framework.agent.ACLMessage;
import yaes.sensornetwork.agents.AbstractSensorAgent;
import yaes.sensornetwork.model.Perception;
import yaes.sensornetwork.model.SensorNetworkWorld;
import yaes.world.physical.location.Location;
import yaes.world.physical.path.PPMTraversal;
import yaes.world.physical.path.PlannedPath;
import yaes.world.physical.path.ProgrammedPathMovement;

public class VCMobileAgent  extends AbstractSensorAgent implements Serializable{
	private PlannedPath plannedpath;
	private PPMTraversal ppmtraversal;

	private Location startLocation;
	private Location terminalLocation;

	
	private boolean enableTraversal = true;
	
	public VCMobileAgent(String name, SensorNetworkWorld sensorWorld) {
		super(name, sensorWorld);
		// TODO Auto-generated constructor stub
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 4206449109512536487L;

	@Override
	public void action(){
		if(!isEnableTraversal()){ //if the path is planned the find next movement location			
			Location currentLoc = ppmtraversal.getLocation(this.getWorld().getTime() - 1.0);
			this.node.setLocation(currentLoc);
		}
		else{ //plan the movement path
			plannedpath.addLocation(startLocation);
			for(int i=0; i<10; i++){ //Adds list of locations along x-axis with 30.0 at fixed y-axis
				plannedpath.addLocation(new Location(20+i, 30));
			}
			plannedpath.addLocation(terminalLocation);
			
			plannedpath.setSource(startLocation);
			plannedpath.setDestination(terminalLocation);
			
			ProgrammedPathMovement ppm = new ProgrammedPathMovement();
			ppm.addSetLocation(startLocation);
	        ppm.addFollowPath(this.plannedpath, 2.0); //set the speed of mobile node as 2.0
	    	this.ppmtraversal = new PPMTraversal(ppm, 0); 
	    	this.enableTraversal = false;
			
		}
	}
	@Override
	protected void handleIntruderPresence(Perception p) {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void handleOverheardMessage(ACLMessage message) {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void handleReceivedMessage(ACLMessage message) {
		// TODO Auto-generated method stub
		
	}
	
	public Location getStartLocation() {
		return startLocation;
	}

	public void setStartLocation(Location startLocation) {
		this.startLocation = startLocation;
	}

	public Location getTerminalLocation() {
		return terminalLocation;
	}

	public void setTerminalLocation(Location terminalLocation) {
		this.terminalLocation = terminalLocation;
	}

	public boolean isEnableTraversal() {
		return enableTraversal;
	}
	public void setEnableTraversal(boolean enableTraversal) {
		this.enableTraversal = enableTraversal;
	}

}
