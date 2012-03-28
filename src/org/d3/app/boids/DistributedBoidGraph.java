package org.d3.app.boids;

import java.net.URI;
import java.util.Collection;
import java.util.LinkedList;
import java.util.concurrent.locks.ReentrantLock;

import org.d3.actor.Feature;
import org.d3.actor.RemoteActor;
import org.d3.annotation.Callable;
import org.graphstream.boids.Boid;
import org.graphstream.boids.BoidGraph;
import org.miv.pherd.geom.Point3;
import org.miv.pherd.geom.Vector3;

public class DistributedBoidGraph extends Feature {

	public static final String CALLABLE_NEAR_OF = "boids.nearOf";
	public static final String CALLABLE_NEW = "boids.new";
	
	BoidGraph localPart;
	ReentrantLock lock;

	protected DistributedBoidGraph(String id) {
		super(id);
	}

	public void initFeature() {
		// TODO Auto-generated method stub

	}

	@Callable(CALLABLE_NEAR_OF)
	public Collection<URI> getBoidsNearOf(Point3 p, Vector3 pDir,
			double viewZone, double angleOfView) {
		LinkedList<Boid> boids = new LinkedList<Boid>();

		for (Boid b : localPart.<Boid> getEachNode()) {
			if (isVisible(p, b.getPosition(), pDir, viewZone, angleOfView))
				boids.add(b);
		}

		return null;
	}

	@Callable(CALLABLE_NEW)
	public void hostNewBoid(String boidId, Point3 position, Vector3 direction) {
		Boid b;
		
		lock.lock();
		b = localPart.addNode(boidId);
		lock.unlock();
		
		b.getForces().setPosition(position.x, position.y, position.z);
		b.getForces().getDirection().copy(direction);
	}
	
	protected boolean isVisible(Point3 boid, Point3 point,
			Vector3 boidDirection, double viewZone, double angleOfView) {
		double d = boid.distance(point);

		if (d <= viewZone) {
			if (angleOfView > -1) {
				double angle;
				Vector3 dir = new Vector3(boidDirection);
				Vector3 light = new Vector3(point.x - boid.x, point.y - boid.y,
						point.z - boid.z);

				dir.normalize();
				light.normalize();

				angle = dir.dotProduct(light);

				if (angle > angleOfView)
					return true;
			} else {
				return true;
			}
		}

		return false;
	}
}
