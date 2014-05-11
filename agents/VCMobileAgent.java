package agents;

import java.io.Serializable;
import java.util.Random;

import yaes.framework.agent.ACLMessage;
import yaes.sensornetwork.agents.AbstractSensorAgent;
import yaes.sensornetwork.model.Perception;
import yaes.sensornetwork.model.SensorNetworkWorld;
import yaes.ui.text.TextUi;
import yaes.virtualcoordinate.VCContext;
import yaes.world.physical.location.Location;
import yaes.world.physical.path.PPMTraversal;
import yaes.world.physical.path.PathGenerator;
import yaes.world.physical.path.PlannedPath;
import yaes.world.physical.path.ProgrammedPathMovement;
import java.awt.geom.Rectangle2D;

public class VCMobileAgent  extends VCAgent implements Serializable{
	private PlannedPath plannedpath;
	private PPMTraversal ppmtraversal;

	private Location startLocation;
	private Location terminalLocation;
	
	//public VCContext context;

	
	private boolean enableTraversal = true;
	
	public VCMobileAgent(String name, VCContext context) {
		super(name, context);
		//this.plannedpath = new PlannedPath();
		//this.startLocation = new Location(0, 0);
		//this.terminalLocation = new Location(0, 0);
		
		//this.context = context;
		// TODO Auto-generated constructor stub
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 4206449109512536487L;
	int time = 0;
	@Override
	public void action(){
		Rectangle2D rectangle = (Rectangle2D)this.getContext().getInterestRectangle();
		if(!isEnableTraversal()){ //if the path is planned the find next movement location			
			Location currentLoc = ppmtraversal.getLocation(time);
			time++;
			this.node.setLocation(currentLoc);
		}
		else{ //plan the movement path
			//double time = this.getWorld().getEndOfTheWorldTime();
			//TextUi.println("end time is: "+time);
			//this.plannedpath = new PlannedPath();
			//this.plannedpath = PathGenerator.createRandomWaypointPathBySegments(new Random(),
		    //        rectangle, (int)(this.getWorld().getEndOfTheWorldTime()*this.getContext().getBaseStation().getSampleTime()*20));
			this.plannedpath = PathGenerator.createRandomWaypointPathBySegments(new Random(),
		            rectangle, 1000);
			this.startLocation = plannedpath.getSource();

			//plannedpath.addLocation(startLocation);
			//for(int i=0; i<10; i++){ //Adds list of locations along x-axis with 30.0 at fixed y-axis
				//plannedpath.addLocation(new Location(20+i, 30));
			//}
			//plannedpath.addLocation(terminalLocation);
			
			//plannedpath.setSource(startLocation);
			//plannedpath.setDestination(terminalLocation);
			
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
