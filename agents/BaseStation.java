package agents;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Ellipse2D.Double;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import yaes.framework.simulation.SimulationInput;
import yaes.sensornetwork.agents.ForwarderSensorAgent;
import yaes.sensornetwork.model.SensorNode;
import yaes.ui.text.TextUi;
import yaes.virtualcoordinate.VCContext;
import Jama.Matrix;
import Jama.SingularValueDecomposition;

public class BaseStation implements Serializable{
	private static final long serialVersionUID = 2281025495542464230L;
	private double sampleTime;
	private Graphics2D detectionEllipse;
	private List<VCAgent> detectionSensors;
	private double timeWindow;
	private VCAgent mobileTarget;
	private ArrayList<VCAgent> neighbourAgents;
	private Matrix mobileVC;
	private double[] currentMobileTC = new double[2];
	private double[] prevMobileTC = new double[2];
	private double[] predictedMobileTC = new double[2];
	private double predictedVelocity = 0;
	private double predictedAngle = 0;
	private HashMap<VCAgent, double[]> TCofNetwork = new HashMap<VCAgent, double[]>();
	private Matrix P;
	private Matrix A;
	private Matrix V;
	private VCContext context;
	
	public BaseStation(SimulationInput sim, VCContext context) {
		this.context = context;
		setSampleTime(sim.getTimeResolution());
		List<VCAgent> myAgents = VCMessageHelper.getAllVCAgents(context.getWorld(), false);
		List<VCAgent> anchorAgents = VCMessageHelper.getAnchorAgents(
				context.getWorld(), ForwarderSensorAgent.class);
		getVCs(myAgents, anchorAgents);
		findTC(myAgents);
	}
	
	public HashMap<VCAgent, double[]> findTC(List<VCAgent> myAgents) {
		Matrix Psvd;
		SingularValueDecomposition B = new SingularValueDecomposition(A);
		V = B.getV().transpose();
		for (int y = 0; y < V.getRowDimension(); y++) {
			V.set(0, y, -V.get(0, y));
			V.set(2, y, -V.get(2, y));
		}
		//for (int x = 0; x < V.getRowDimension(); x++) {
			//for (int y = 0; y < V.getColumnDimension(); y++) {
				//TextUi.print(Double.toString(V.get(x, y))+"  ");
			//}
			//TextUi.println("");
		//}
		//TextUi.println("");
		//for (int x = 0; x < A.getRowDimension(); x++) {
			//for (int y = 0; y < A.getColumnDimension(); y++) {
				//TextUi.print(Double.toString(A.get(x, y))+"  ");
			//}
			//TextUi.println("");
		//}
		//V = B.getV();
		Psvd = P.times(V);

		for (int i  = 0; i < myAgents.size(); i++) {
			double thetaT = Math.atan( (Psvd.get(i, 2)) / (Psvd.get(i, 1)));
			double rT = Math.sqrt(Math.pow(Psvd.get(i, 0), 2) + 
								  Math.pow(Psvd.get(i, 1), 2) + 
								  Math.pow(Psvd.get(i, 2), 2) );
			double[] axis = {rT * Math.cos(thetaT), rT * Math.sin(thetaT)};
			TCofNetwork.put(myAgents.get(i), axis);
			
		}
		return TCofNetwork;
		
		
	}
	
	public double[] sample() {
		Matrix Msvd;
		//TextUi.println(mobileVC.getColumnDimension() + " " + V.getColumnDimension());
		Msvd = mobileVC.transpose().times(V);
		double[][] temp = mobileVC.getArray();
		TextUi.println("Node is at VC " + temp[0][0] +" " + temp[1][0] +" " + temp[2][0] +" " + temp[3][0]);
		//TextUi.println(Msvd)
		double thetaT = Math.atan( (Msvd.get(0, 2)) / (Msvd.get(0, 1)));
		double rT = Math.sqrt(Math.pow(Msvd.get(0, 0), 2) + 
				  Math.pow(Msvd.get(0, 1), 2) + 
				  Math.pow(Msvd.get(0, 2), 2) );
		prevMobileTC[0] = currentMobileTC[0];
		prevMobileTC[1] = currentMobileTC[1];
		currentMobileTC[0] = rT * Math.cos(thetaT);
		currentMobileTC[1] = rT * Math.sin(thetaT);
		TextUi.println("TCs of Node are "+currentMobileTC[0] + " " + currentMobileTC[1]);
		return currentMobileTC;
	}
	
	public double[] predictTC() {
		
		double temp = Math.sqrt(Math.pow((currentMobileTC[0] - prevMobileTC[0]), 2) +
								Math.pow((currentMobileTC[1] - prevMobileTC[1]), 2));
		predictedVelocity = temp/sampleTime;
		
		predictedAngle = Math.acos((currentMobileTC[0] - prevMobileTC[0])/temp);
		
		predictedMobileTC[0] = currentMobileTC[0] + predictedVelocity * sampleTime * Math.cos(predictedAngle);
		predictedMobileTC[1] = currentMobileTC[1] + predictedVelocity * sampleTime * Math.sin(predictedAngle);
		
		//prevMobileTC[0] = currentMobileTC[0];
		//prevMobileTC[1] = currentMobileTC[1];
		return predictedMobileTC;
	}
	
	public Ellipse2D.Double getEllipse(double x, double y, double width, double height, double rotation) {
	    double newX = x - width / 2.0;
	    double newY = y - height / 2.0;

	    Ellipse2D.Double ellipse = new Ellipse2D.Double(newX, newY, width, height);
	    Shape sh = AffineTransform.getRotateInstance(rotation + Math.PI)
	    .createTransformedShape(ellipse);

	    return (Ellipse2D.Double)sh;
	}
	
	public void alertESernsors(){
		
	}
	
	private void getVCs(List<VCAgent> myAgents, List<VCAgent> anchorAgents){
		
		double[][] VCs = new double[myAgents.size()][anchorAgents.size()];
		double[][] AVCs = new double[anchorAgents.size()][anchorAgents.size()];
		
		int i = 0;
		VCLocalTable table = context.getCorrectVCLocalTable();
		for (VCAgent anchor : anchorAgents) {
			int j = 0;
			int k = 0;
			for (VCAgent agent : myAgents) {
				
				VCs[j][i] = table.getNumberOfHops(agent, anchor);
				//System.out.println(VCs[j][i]);
				j++;
				if (agent.isAnchor()) {
					AVCs[k][i] = table.getNumberOfHops(agent, anchor);
					k++;
				}
			}
			i++;
		}
		//for (int l = 0; l < anchorAgents.size(); l++) {
			//for (int m = 0; m < anchorAgents.size(); m++) {
				//TextUi.print(Double.toString(AVCs[l][m])+"  ");
			//}
			//TextUi.println("");
		//}

		P = new Matrix(VCs);
		A = new Matrix(AVCs);
	}
	//at each time step multiple of sample time set the currentMobileTC value in update function
	
	
	public double[] getPrevTC() {
		return prevMobileTC;
	}
	
	public double[] getFutureTC() {
		return predictedMobileTC;
	}
	
	public double[] getCurrentTC() {
		return currentMobileTC;
	}
	
	public double getPredictedAngle() {
		return predictedAngle;
	}
	
	public void setMobileVC(double[] VC, int anchorsize) {
		mobileVC = new Matrix(VC, anchorsize);
	}
	
	private void setSampleTime(double time) {
		sampleTime = time;
	}
	
	public void setPrevTC() {
		prevMobileTC[0] = currentMobileTC[0];
		prevMobileTC[1] = currentMobileTC[1];
	}
	
	public HashMap<VCAgent, double[]> getTCofNetwork() {
		return TCofNetwork;
	}
	

}
