#!/bin/bash

if [ -z "$1" ]; then
	CFG="d3-config.cfg"
else
	CFG="$1"
fi

M2="$HOME/.m2/repository"
LIBS="$M2/org/graphstream/gs-core/1.2-git/gs-core-1.2-git.jar \
$M2/org/graphstream/gs-algo/1.2-git/gs-algo-1.2-git.jar \
$M2/org/graphstream/gs-boids/git/gs-boids-git.jar \
$M2/org/d3/d3-core/1.0-git/d3-core-1.0-git.jar \
$M2/org/javassist/javassist/3.14.0-GA/javassist-3.14.0-GA.jar \
$M2/org/graphstream/pherd/1.0/pherd-1.0.jar \
$M2/org/graphstream/mbox2/1.0/mbox2-1.0.jar"
MAIN="org.d3.app.boids.StartBoid"
JVM_OPTS="-Dorg.d3.config=$CFG -Dsun.java2d.opengl=true"
CLASSPATH="bin/"

for LIB in $LIBS; do
	CLASSPATH+=":$LIB"
done

echo $CLASSPATH
java $JVM_OPTS -cp $CLASSPATH $MAIN
