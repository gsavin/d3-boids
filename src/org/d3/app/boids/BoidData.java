package org.d3.app.boids;

import java.io.Serializable;

import org.miv.pherd.geom.Point3;
import org.miv.pherd.geom.Vector3;

public class BoidData implements Serializable {
	private static final long serialVersionUID = 6561361450638974214L;

	String boidId;
	String speciesName;
	Point3 position;
	Vector3 direction;
	
	public String getBoidId() {
		return boidId;
	}
	
	public Point3 getPosition() {
		return position;
	}
	
	public Vector3 getDirection() {
		return direction;
	}
	
	public String getSpeciesName() {
		return speciesName;
	}
}
