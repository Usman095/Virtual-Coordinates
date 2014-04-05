package util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import yaes.framework.simulation.SimulationInput;
import yaes.ui.text.TextUi;
import yaes.virtualcoordinate.VCConstants;
import yaes.world.physical.location.Location;

/**
 * This class scans various files and return the required
 * format. For example, it returns the list of locations
 * required for the CSU benchmarks
 * @author Saad Khan
 *
 */
public class ScanFile implements VCConstants, Serializable{
	private static final long serialVersionUID = -9214931623495676416L;
	private final File scannedFile;
	SimulationInput sip;

	private Scanner scanner;
	public ScanFile(String aFileName, SimulationInput sip){
	    scannedFile = new File(aFileName);
	    this.sip = sip;
	}

	public final void processLineByLine() throws FileNotFoundException {
	    Scanner scanner = new Scanner(new FileReader(scannedFile));
	    try {
	      while ( scanner.hasNextLine() )
	    	  processLine( scanner.nextLine() );
	    }
	    finally {
	      scanner.close();
	    }
 	  }

	
	public List<Location> processLocationList() throws FileNotFoundException {
		List<Location> locList = new ArrayList<Location>();
	    Scanner scanner = new Scanner(new FileReader(scannedFile));
	    try {
	      while ( scanner.hasNextLine() )
	    	locList.add(processCoordinateLine(scanner.nextLine()));
	    }
	    finally {
	      scanner.close();
	    }
	    return locList;
 	  }
	
	  protected void processLine(String aLine){
	    scanner = new Scanner(aLine);
	    scanner.useDelimiter("\t");
	    if ( scanner.hasNext() ){
	      String x = scanner.next();
	      String y = scanner.next();
	      TextUi.print("x: " + x.trim() + ", y: " + y.trim());
	    }
	    else {
	    	TextUi.print("Empty or invalid line. Unable to process.");
	    }
	  }
	  
	  protected Location processCoordinateLine(String aLine){
		  Location retVal = new Location(0, 0);
		  scanner = new Scanner(aLine);
		  scanner.useDelimiter("\t");
		  if ( scanner.hasNext() )
		  	retVal = new Location(Double.valueOf(scanner.next()), Double.valueOf(scanner.next()));
		  else 
		   	TextUi.print("Empty or invalid line. Unable to process.");
		  return retVal;
	  }	  		  
}
