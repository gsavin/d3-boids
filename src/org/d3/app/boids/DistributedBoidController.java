package org.d3.app.boids;

import java.util.concurrent.TimeUnit;

import org.d3.Console;
import org.d3.actor.Feature;
import org.d3.actor.StepActor;

public class DistributedBoidController extends Feature implements StepActor {

	DistributedBoidGraph ctx;
	long averageStepTime;
	long lastOutput;
	int step;

	public DistributedBoidController(DistributedBoidGraph ctx) {
		super(ctx.getId() + "-controller");
		this.ctx = ctx;
		this.averageStepTime = 0;
		this.step = 0;
		this.lastOutput = 0;
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

		averageStepTime += m2 - m1;
		step++;

		if (m2 - lastOutput > 1000) {
			Console.info("average step time is %dms", averageStepTime / step);
			lastOutput = m2;
		}
	}

}
