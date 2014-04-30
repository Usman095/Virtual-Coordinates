package util;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.io.Serializable;

import yaes.sensornetwork.model.SensorNetworkWorld;
import yaes.sensornetwork.model.SensorNode;
import yaes.sensornetwork.visualization.paintSensorNode;
import yaes.ui.visualization.VisualCanvas;
import yaes.ui.visualization.VisualizationProperties;
import yaes.ui.visualization.painters.PainterHelper;
import yaes.world.physical.location.IMoving;
import agents.VCAgent;

public class PaintVCNode extends paintSensorNode implements Serializable{

	private static final long serialVersionUID = 3763301612027110720L;

	public PaintVCNode(SensorNetworkWorld sensorNetworkWorld) {
		super(sensorNetworkWorld);
	}

	@Override
	public void paint(Graphics2D g, Object o, VisualCanvas panel) {
		VisualizationProperties vprops = panel.getVisualizer()
				.getVisualizationProperties();
		if (!(o instanceof SensorNode)) {
			throw new Error("This is supposed to handle a SensorNode!!!");
		}
		final SensorNode node = (SensorNode) o;
		final VCAgent agent = (VCAgent) node.getAgent();
		// let us call the upper one

		if (panel.getCurrentLayer() == FOREGROUND_LAYER) {
			// default 
			size = 0;
			super.paint(g, o, panel);
			// the node had been painted already, here we paint over it
			IMoving wl = (IMoving) o;
			if (agent.isAnchor()) {
				Ellipse2D.Double ellipse = new Ellipse2D.Double(0,0,18,18);
				PainterHelper.paintShapeAtLocation(wl.getLocation(), ellipse, Color.black, Color.black, g, panel);
				PainterHelper.paintRectangleAtLocation(wl.getLocation(), 10,
						Color.white, Color.white, g, panel);
			} else if (agent.isInLocalArea()) {
				Ellipse2D.Double ellipse = new Ellipse2D.Double(0,0,12,12);
				PainterHelper.paintShapeAtLocation(wl.getLocation(), ellipse, Color.black, Color.black, g, panel);
				//PainterHelper.paintRectangleAtLocation(wl.getLocation(), 20,
				//		Color.black, Color.gray, g, panel);
			} else if (agent.isInPreviousLocalArea()) {
				Ellipse2D.Double ellipse = new Ellipse2D.Double(0,0,12,12);
				PainterHelper.paintShapeAtLocation(wl.getLocation(), ellipse, Color.black, Color.gray.brighter(), g, panel);
				//PainterHelper.paintRectangleAtLocation(wl.getLocation(), 15,
				//		Color.black, Color.gray, g, panel);
				//((Graphics2D)ellipse).rotate(arg0, arg1, arg2)
			}
			else if (agent.getNode().getName().equals("MobileNode")){
				Ellipse2D.Double ellipse = new Ellipse2D.Double(0,0,10,10);
				PainterHelper.paintShapeAtLocation(wl.getLocation(), ellipse, Color.black, Color.red, g, panel);
				//PainterHelper.paintRectangleAtLocation(wl.getLocation(), 10,
				//		Color.black, Color.gray, g, panel);
			}
			else {
				Ellipse2D.Double ellipse = new Ellipse2D.Double(0,0,10,10);
				PainterHelper.paintShapeAtLocation(wl.getLocation(), ellipse, Color.black, Color.white, g, panel);
			}

		}
		// routes go to the communication events layer
		if (panel.getCurrentLayer() == COMMUNICATION_LINKS_LAYER) {
			super.paint(g, o, panel);
		}
	}

}
