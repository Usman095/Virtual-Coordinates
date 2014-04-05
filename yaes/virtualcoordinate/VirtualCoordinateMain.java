package yaes.virtualcoordinate;

import java.awt.Toolkit;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import yaes.Version;
import yaes.framework.simulation.Simulation;
import yaes.framework.simulation.SimulationInput;
import yaes.framework.simulation.parametersweep.ExperimentPackage;
import yaes.framework.simulation.parametersweep.ParameterSweep;
import yaes.framework.simulation.parametersweep.ParameterSweep.ParameterSweepType;
import yaes.framework.simulation.parametersweep.ParameterSweepHelper;
import yaes.framework.simulation.parametersweep.ScenarioDistinguisher;
import yaes.sensornetwork.constSensorNetwork;
import yaes.ui.simulationcontrol.SimulationReplayTxt;
import yaes.ui.text.TextUi;

/**
 * This is the main file for the Virtual Coordinate Sensor Network
 * 
 * @author Saad Khan
 * 
 */
public class VirtualCoordinateMain implements Serializable, VCConstants {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1555982717405826321L;
	private static final String MENU_SETUP_RUN = "Virtual Coordinate system - Setup the Virtual Coordinate System";
	private static final String MENU_DVCS_RUN = "Virtual Coordinate system - Run the Directional Forwarding VCS";
	private static final String MENU_DVCS_MOB_SINK_RUN = "Virtual Coordinate system - Run the Directional Forwarding towards Mobile Sink";
	private static final String MENU_VC_VISUAL = "Virtual Coordinate system - visualize run";
	private static final String MENU_VC_GRAPHS = "Virtual Coordinate system - full simulation with graphs run";
	private static final String MENU_TCTP_RUN = "Virtual Coordinate system - Run Mobile Target Tracking";

	public static final File outputDir = new File(
			"data/VirtualCoordinate/output");
	public static final File graphDir = new File(
			"data/VirtualCoordinate/graphs");
	public static final File logDir = new File("data/VirtualCoordinate/log");
	public static final File cache = new File("data/VirtualCoordinate/cache");

	static {
		outputDir.mkdirs();
		graphDir.mkdirs();
		logDir.mkdirs();
		cache.mkdirs();
	}

	public static void main(String[] args) throws Exception {
		TextUi.println(Version.versionString());
		final List<String> menu = new ArrayList<String>();
		String result = null;
		String defaultChoice = VirtualCoordinateMain.MENU_SETUP_RUN;
		menu.add(VirtualCoordinateMain.MENU_SETUP_RUN);
		menu.add(VirtualCoordinateMain.MENU_DVCS_RUN);
		menu.add(VirtualCoordinateMain.MENU_DVCS_MOB_SINK_RUN);
		menu.add(VirtualCoordinateMain.MENU_TCTP_RUN);
		menu.add(VirtualCoordinateMain.MENU_VC_VISUAL);
		menu.add(VirtualCoordinateMain.MENU_VC_GRAPHS);
		if (result == null) {
			result = TextUi.menu(menu, defaultChoice, "Choose:");
		}
		switch (result) {
		case VirtualCoordinateMain.MENU_SETUP_RUN: {
			doSimpleRun();
			break;
		}
		case VirtualCoordinateMain.MENU_DVCS_RUN: {
			doDVCSRun();
			break;
		}
		case VirtualCoordinateMain.MENU_DVCS_MOB_SINK_RUN: {
			doDVCSMobileSinkRun();
			break;
		}
		case VirtualCoordinateMain.MENU_TCTP_RUN: {
			doTCTPRun();
			break;
		}
		case VirtualCoordinateMain.MENU_VC_VISUAL: {
			SimulationReplayTxt sr = new SimulationReplayTxt(logDir);
			sr.mainLoop();
			break;
		}
		case VirtualCoordinateMain.MENU_VC_GRAPHS: {
			runFullSimulation();
			break;
		}
		default:
			break;
		}
		Toolkit.getDefaultToolkit().beep();
		TextUi.println("Done, exiting");
		System.exit(0);
	}

	/**
	 * Creates the default simulation input
	 * 
	 * @return
	 */
	public static SimulationInput createDefaultSimulationInput() {
		SimulationInput model = new SimulationInput();
		model.setContextClass(VCContext.class);
		model.setSimulationClass(VCSimulation.class);
		model.setParameter(SensorAgentClass.VirtualCoordinate);
		model.setParameter(NetworkMode.DVCR_TO_MOBILE_SINK);
		model.setParameter(CSUBenchmark.CIRCULAR3VOID);
		model.setParameter(constSensorNetwork.SensorDeployment_SinkNodeX, 300.0);
		model.setParameter(constSensorNetwork.SensorDeployment_SinkNodeY, 300.0);

		model.setParameter(
				constSensorNetwork.SensorDeployment_InterestRectangleX, 0.0);
		model.setParameter(
				constSensorNetwork.SensorDeployment_InterestRectangleY, 0.0);
		model.setParameter(
				constSensorNetwork.SensorDeployment_InterestRectangleWidth,
				400.0);
		model.setParameter(
				constSensorNetwork.SensorDeployment_InterestRectangleHeight,
				400.0);

		model.setParameter(constSensorNetwork.SensorArrangement.RANDOM);

		model.setParameter(SinkMobilityMode.MOBILE);
		model.setParameter(SinkMovementType.RANDOM_WAYPOINT);
		model.setParameter(SinkMoveNotificationType.CIRCLE);
		model.setParameter(constSensorNetwork.SensorDeployment_SensorRange,
				150.0);
		model.setParameter(
				constSensorNetwork.SensorDeployment_TransmissionRange, 30.0);
		model.setParameter(constSensorNetwork.SensorDeployment_SensorNodeCount,
				3000);
		model.setParameter(
				constSensorNetwork.Scenario_IntruderMovementRandomSeed, 133);
		model.setParameter(VCConstants.AnchorPlacementMode.EXTREMES);
		model.setParameter(VCConstants.MessagesToBeGenerated, 3000);
		model.setParameter(VCConstants.UseDensity, 1);
		model.setParameter(VCConstants.NetworkDensity, 0.005);
		model.setParameter(VCConstants.TTLCount, 100);
		model.setParameter(VCConstants.SINK_MOVE_NOTIFY_RADIOUS, 9);
		model.setParameter(VCConstants.SINK_MOVE_DELAY, 30);
		model.setParameter(VCConstants.AnchorCount, 4);
		model.setParameter(VCConstants.NetworkScale, 1.0);
		model.setParameter(VCConstants.MessagesGenerated, 0);
		model.setParameter(VCConstants.SuccessfullRoutings, 0);
		model.setParameter(VCConstants.RoutedMessages, 0);
		model.setParameter(VCConstants.FailedRoutings, 0);
		model.setParameter(VisualDisplay.NO);
		model.setParameter(SimControl_KeepTimeSeries, 0);
		return model;
	}

	/**
	 * Performs a sample run of the value of deep inspection scenario
	 * 
	 * @param model
	 * @throws IllegalAccessException
	 * @throws InstantiationException
	 * @throws IOException
	 */
	public static void doSimpleRun() throws InstantiationException,
			IllegalAccessException, IOException {
		final SimulationInput sip = createDefaultSimulationInput();
		sip.setStopTime(150000);
		final VCContext context = new VCContext();
		// logDir.mkdirs();
		sip.setParameter(NetworkMode.VC_INITILIZE_SETUP);
		sip.setParameter(VisualDisplay.NO);
		Simulation.simulate(sip, VCSimulation.class, context, logDir);
	}

	/**
	 * This method runs the directional forwarding virtual coordinate system
	 * 
	 * @throws InstantiationException
	 * @throws IllegalAccessException
	 * @throws IOException
	 */
	public static void doDVCSRun() throws InstantiationException,
			IllegalAccessException, IOException {
		final SimulationInput sip = createDefaultSimulationInput();
		sip.setStopTime(1000);
		sip.setParameter(NetworkMode.DIRECTIONAL_VC);
		sip.setParameter(SinkMobilityMode.STATIC);
		final VCContext context = new VCContext();
		// logDir.mkdirs();
		sip.setParameter(VisualDisplay.NO);
		Simulation.simulate(sip, VCSimulation.class, context, logDir);
	}

	/**
	 * This method runs the directional forwarding virtual coordinate system
	 * 
	 * @throws InstantiationException
	 * @throws IllegalAccessException
	 * @throws IOException
	 */
	public static void doDVCSMobileSinkRun() throws InstantiationException,
			IllegalAccessException, IOException {
		final SimulationInput sip = createDefaultSimulationInput();
		sip.setStopTime(1000);
		sip.setParameter(NetworkMode.DVCR_TO_MOBILE_SINK);
		sip.setParameter(SinkMobilityMode.MOBILE);
		final VCContext context = new VCContext();
		// logDir.mkdirs();
		sip.setParameter(VisualDisplay.NO);
		Simulation.simulate(sip, VCSimulation.class, context, logDir);
	}
	
	/**
	 * This method runs the Topological Coordinate Tracking and Prediction
	 * 
	 * @throws InstantiationException
	 * @throws IllegalAccessException
	 * @throws IOException
	 */
	public static void doTCTPRun() throws InstantiationException,
			IllegalAccessException, IOException {
		final SimulationInput sip = createDefaultSimulationInput();
		sip.setStopTime(150000);
		sip.setParameter(NetworkMode.TCTP);
		sip.setParameter(SinkMobilityMode.STATIC);
		final VCContext context = new VCContext();
		// logDir.mkdirs();
		sip.setParameter(VisualDisplay.NO);
		Simulation.simulate(sip, VCSimulation.class, context, logDir);
	}

	/**
	 * @param model
	 * @throws ClassNotFoundException
	 * @throws IllegalAccessException
	 * @throws InstantiationException
	 * @throws IOException
	 * @throws FileNotFoundException
	 */
	public static void runFullSimulation() throws FileNotFoundException,
			IOException, InstantiationException, IllegalAccessException,
			ClassNotFoundException {
		SimulationInput model = createDefaultSimulationInput();
		model.setStopTime(150000);
		compareVariableIntruderNodes(model);
	}

	/**
	 * @param model
	 * @throws IOException
	 * @throws FileNotFoundException
	 * @throws ClassNotFoundException
	 * @throws IllegalAccessException
	 * @throws InstantiationException
	 */
	private static void compareVariableIntruderNodes(SimulationInput model)
			throws FileNotFoundException, IOException, InstantiationException,
			IllegalAccessException, ClassNotFoundException {
		ExperimentPackage pack = new ExperimentPackage(outputDir, graphDir);
		pack.setModel(model);
		ParameterSweep sweepDiscrete = getAgentTypes();
		pack.addParameterSweep(sweepDiscrete);
		// parameterized sweep

//		 ParameterSweep nofiticationRadious = ParameterSweepHelper
//		 .generateParameterSweepInteger("Radious",
//		 VCConstants.SINK_MOVE_NOTIFY_RADIOUS, 5, 25, 3);
//		 pack.addParameterSweep(nofiticationRadious);

		ParameterSweep networkSideLength = ParameterSweepHelper
				.generateParameterSweepDouble(
						"Side Length (m)",
						constSensorNetwork.SensorDeployment_InterestRectangleWidth,
						5, 400, 1200);
		pack.addParameterSweep(networkSideLength);

//		ParameterSweep sinkSpeed = ParameterSweepHelper
//				 .generateParameterSweepInteger("Sink Speed",
//				 VCConstants.SINK_MOVE_DELAY, 10, 50, 10);
//				 pack.addParameterSweep(sinkSpeed);
				 
				 
		ParameterSweep sweepRandom = ParameterSweepHelper
				.generateParameterSweepInteger("Random",
						constSensorNetwork.Scenario_IntruderMovementRandomSeed,
						131, 135);
		sweepRandom.setType(ParameterSweepType.Repetition);
		pack.addParameterSweep(sweepRandom);

		// ParameterSweep sweepIntruders = ParameterSweepHelper
		// .generateParameterSweepInteger("Intruder nodes",
		// constSensorNetwork.Scenario_IntruderNodeCount, 1, 40, 5);
		// pack.addParameterSweep(sweepIntruders);
		// pack.cleanUp();
		pack.initialize();
		pack.run(false);
		pack.setVariableDescription(
				"DVCR",
				"UA-DVCR");
		pack.setVariableDescription(
				constSensorNetwork.SensorDeployment_InterestRectangleWidth,
				"Area width (m)");
		pack.setVariableDescription(
				constSensorNetwork.Metrics_MessagesSentSum,
				"Number of messages");
		pack.setVariableDescription(
				constSensorNetwork.Metrics_TransmissionEnergySum,
				"Energy Consumption (J)");
		pack.generateGraph(Metrics_MessagesSentSum, "Number of sent messages",
				"number_of_sent_messages");
		pack.generateGraph(Metrics_MessagesGeneratedSum,
				"Number of generated messages", "number_of_generated_messages");
		pack.generateGraph(Metrics_SuccessfullRoutings,
				"Number of successfull routings",
				"number_of_successfull_routings");
		pack.generateGraph("Metrics_TransmissionEnergySum",
				"Energy Consumption (J)", "transmission_energy");
		pack.generateGraph(Metrics_AveragePathLength, null,
				"average_path_length");
		pack.generateGraph(Metrics_RoutedMessages, "Number of routed messages",
				"number_of_routed_messages");
		pack.generateGraph(Metrics_AverageNeighborsCount,
				"average neighbors count", "average_neighbors_count");

	}

	/**
	 * We are comparing across different benchmarks of Colorado State University
	 * protection algorithm
	 * 
	 * @return
	 */
	public static ParameterSweep getAgentTypes() {
		ParameterSweep sweepDiscrete = new ParameterSweep("NodeDistrbution");
		ScenarioDistinguisher sd = null;

		// Add the circular three void scenario

		// sd = new ScenarioDistinguisher("Notify-Broadcast");
		// sd.setDistinguisher(SinkMoveNotificationType.BROADCAST);
		// sweepDiscrete.addDistinguisher(sd);

		sd = new ScenarioDistinguisher("DVCR");
		sd.setDistinguisher(SinkMoveNotificationType.BROADCAST);
		sd.setDistinguisher(SINK_MOVE_NOTIFY_RADIOUS, 1000);
		sweepDiscrete.addDistinguisher(sd);
		
		sd = new ScenarioDistinguisher("MS-DVCR");
		sd.setDistinguisher(SinkMoveNotificationType.BROADCAST);
		sweepDiscrete.addDistinguisher(sd);
		
		sd = new ScenarioDistinguisher("CU-MS-DVCR");
		sd.setDistinguisher(SinkMoveNotificationType.CIRCLE);
		sweepDiscrete.addDistinguisher(sd);
		 
		// sd = new ScenarioDistinguisher("Scale=2");
		// sd.setDistinguisher(CSUBenchmark.CIRCULAR3VOID);
		// sd.setDistinguisher(VCConstants.NetworkSale, Math.sqrt(2.205));
		// sweepDiscrete.addDistinguisher(sd);
		// sd = new ScenarioDistinguisher("Scale=4");
		// sd.setDistinguisher(CSUBenchmark.CIRCULAR3VOID);
		// sd.setDistinguisher(VCConstants.NetworkSale, Math.sqrt(4.157));
		// sweepDiscrete.addDistinguisher(sd);
		return sweepDiscrete;
	}

}
