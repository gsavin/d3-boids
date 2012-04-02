package org.d3.app.boids;

import java.io.Serializable;
import java.net.URI;

import org.miv.pherd.geom.Point3;
import org.miv.pherd.geom.Vector3;

public class BoidData implements Serializable {
	private static final long serialVersionUID = 6561361450638974214L;
	
	String boidId;
	String speciesName;
	Point3 position;
	Vector3 direction;
	URI uri;

	public BoidData(String id, String species) {
		boidId = id;
		speciesName = species;
		position = new Point3();
		direction = new Vector3();
	}

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

	public void setURI(URI uri) {
		this.uri = uri;
	}

	public URI getURI() {
		return uri;
	}
}
