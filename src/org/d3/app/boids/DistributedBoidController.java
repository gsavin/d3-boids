package org.d3.app.boids;

import java.util.concurrent.TimeUnit;

import org.d3.Console;
import org.d3.actor.Feature;
import org.d3.actor.StepActor;

public class DistributedBoidController extends Feature implements StepActor {

	DistributedBoidGraph ctx;

	public DistributedBoidController(DistributedBoidGraph ctx) {
		super(ctx.getId() + "-controller");
		this.ctx = ctx;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.Feature#initFeature()
	 */
	public void initFeature() {
		Console.info("init ok");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.StepActor#getStepDelay(java.util.concurrent.TimeUnit)
	 */
	public long getStepDelay(TimeUnit unit) {
		return unit.convert(ctx.getSleepTime(), TimeUnit.MILLISECONDS);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.d3.actor.StepActor#step()
	 */
	public void step() {
		long m1, m2;
		m1 = System.currentTimeMillis();
		ctx.step();
		m2 = System.currentTimeMillis();

		Console.info("step in %dms", m2 - m1);
	}

}
