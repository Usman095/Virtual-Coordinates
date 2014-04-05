package test.virtualcoordinate;

import java.io.FileNotFoundException;
import java.io.Serializable;
import java.util.List;

import org.junit.Test;

import util.ScanFile;
import yaes.framework.simulation.SimulationInput;
import yaes.ui.text.TextUi;
import yaes.world.physical.location.Location;

public class testParseFile implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = 2847065536457242076L;

	@Test
	public void test() throws FileNotFoundException {
	  	ScanFile parser = new ScanFile("datasets/CircularVoidNetwork.txt", new SimulationInput());
	   List<Location> retVal = parser.processLocationList(); 
	    for(Location loc: retVal){
	    	 TextUi.print(loc.toString());
	    }
	    TextUi.print("Done.");
	}
}


