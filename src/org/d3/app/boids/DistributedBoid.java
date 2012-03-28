package org.d3.app.boids;

import org.d3.actor.Entity;
import org.d3.entity.Migratable;
import org.graphstream.boids.Boid;
import org.miv.pherd.geom.Point3;
import org.miv.pherd.geom.Vector3;

public class DistributedBoid extends Entity {
	private static final long serialVersionUID = 254153604563418868L;

	protected Boid boid;

	@Migratable
	protected Point3 position;
	
	@Migratable
	protected Vector3 direction;
	
	protected DistributedBoid(String id) {
		super(id);
		
		position = null;
		direction = null;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.Entity#initEntity()
	 */
	@Override
	public void initEntity() {
		
	}
}
