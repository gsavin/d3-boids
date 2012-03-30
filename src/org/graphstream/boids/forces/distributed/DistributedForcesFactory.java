package org.graphstream.boids.forces.distributed;

import org.d3.app.boids.DistributedBoidGraph;
import org.graphstream.boids.Boid;
import org.graphstream.boids.BoidForces;
import org.graphstream.boids.BoidForcesFactory;
import org.miv.pherd.geom.Point3;

public class DistributedForcesFactory implements BoidForcesFactory {

	DistributedBoidGraph localPart;

	public DistributedForcesFactory(DistributedBoidGraph local) {
		localPart = local;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * org.graphstream.boids.BoidForcesFactory#createNewForces(org.graphstream
	 * .boids.Boid)
	 */
	public BoidForces createNewForces(Boid b) {
		return new DistributedForces(b);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.graphstream.boids.BoidForcesFactory#end()
	 */
	public void end() {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.graphstream.boids.BoidForcesFactory#step()
	 */
	public void step() {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * org.graphstream.boids.BoidForcesFactory#resize(org.miv.pherd.geom.Point3,
	 * org.miv.pherd.geom.Point3)
	 */
	public void resize(Point3 low, Point3 high) {
	}
}
