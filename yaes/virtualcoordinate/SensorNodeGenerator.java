package yaes.virtualcoordinate;

import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import yaes.framework.simulation.SimulationInput;
import yaes.virtualcoordinate.VCConstants.CSUBenchmark;
import yaes.world.physical.location.Location;

/**
 * 
 * @author Rouhollah
 *
 */

public class SensorNodeGenerator implements Serializable{

	/**
	 * 
	 */
	private static final long serialVersionUID = 5870714044504719651L;
	
	public static List<Location> generateNodes(SimulationInput sip, double scale) {
		List<Location> locationList = new ArrayList<Location>();
		switch(sip.getParameterEnum(CSUBenchmark.class)){
			default:
				// 0.000001s are for handling boundaries
				Area area = new Area(new Ellipse2D.Double(0.000001, 0.000001, 30, 30));
		        area.exclusiveOr(new Area(new Ellipse2D.Double(6 - 0.000001, 16 - 0.000001, 8 + .00001, 8 + .00001)));
		        area.exclusiveOr(new Area(new Ellipse2D.Double(6 - 0.000001, 6 - 0.000001, 8 + .00001, 8 + .00001)));
		        area.exclusiveOr(new Area(new Ellipse2D.Double(16 - 0.000001, 9 - 0.000001, 12 + .00001, 12 + .00001)));
		        AffineTransform at = new AffineTransform();
		        at.concatenate(AffineTransform.getScaleInstance(scale, scale));
		        area.transform(at);
		        for(int x = 0; x < 31*scale; x++) {
		        	for(int y = 0; y < 31*scale; y++) {
		        		if(area.contains(x, y)) {
		        			locationList.add(new Location(x, y));
		        		}
		        	}
				}
			break;			
		}
		
		return locationList;
	}
}
