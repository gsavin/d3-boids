#!/bin/bash

if [ -z "$1" ]; then
	CFG="d3-config.cfg"
else
	CFG="$1"
fi

M2="/home/raziel/.m2/repository"
ECLIPSE_PROJECT="gs-core gs-boids d3"
LIBS="../d3/lib/javassist.jar $M2/org/graphstream/pherd/1.0/pherd-1.0.jar $M2/org/graphstream/mbox2/1.0/mbox2-1.0.jar"
MAIN="org.d3.app.boids.StartBoid"
JVM_OPTS="-Dorg.d3.config=$CFG -Dsun.java2d.opengl=true"
CLASSPATH="bin/"

for PROJECT in $ECLIPSE_PROJECT; do
	CLASSPATH+=":../$PROJECT/bin/"
done

for LIB in $LIBS; do
	CLASSPATH+=":$LIB"
done

echo $CLASSPATH
java $JVM_OPTS -cp $CLASSPATH $MAIN
