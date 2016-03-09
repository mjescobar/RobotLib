#include <ROBOTLIB>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <time.h>

using namespace std;

int main()
{
	srand (time(NULL));
	int numMotores = 12;
	vector <Joint *> jointVector; 
	vector < CollisionObject * > collisionObjectVector;
	RobotVREP * simulador = new RobotVREP(true);
	simulador->changeVideoPath((char *)"/home/oscar/Documentos/GIT USM/RobotLib/prueba/video");


	// Agregando los motores.
	for(int i = 1; i < numMotores+1; i++)
	{
		stringstream jointName;
		jointName << "joint" << i << "#";
		Joint * joint  = new Joint((char *)jointName.str().c_str(), M_PI/4.0, -M_PI/4.0, 0.0,(char *)"SCALE" );
		jointVector.push_back( joint );		
		simulador->addJoint( joint );
	}

	for(int i = 12; i < 16; i++)
	{
		stringstream collisionObjectName;
		collisionObjectName << "Collision" << i;
		CollisionObject * collisionObject  = new CollisionObject((char *)collisionObjectName.str().c_str());
		collisionObjectVector.push_back( collisionObject );		
		simulador->addCollisionObject( collisionObject );
	}

	// Agregando los sensores

	Object * argoCenter = new Object((char *)"ArgoV2_reference");

	simulador->addObject(argoCenter);

	int timer = 0;
	simulador->startSimulation();
	while( timer < 100 )
	{
		for (unsigned int i = 0; i < jointVector.size(); ++i)
		{
			double delta = (2*(rand()/(double)(RAND_MAX)) -1 )*0.1; 
			jointVector.at(i)->setJointNextPosition( jointVector.at(i)->getJointGoalPosition() + delta );
		}

		cout << "Collisions:\t";
		for(int i = 0; i < (int)collisionObjectVector.size(); i++)
		{
			cout << ((simulador->readCollision(collisionObjectVector.at(i))) ? 1 : 0) << "\t";
		}
		cout << endl;

		simulador->moveJoints();
		std::vector <double> position;
		position = simulador->getObjectPosition(argoCenter);
		cout << "Argo position: " << position.at(0) << ", " << position.at(1) << ", " << position.at(2) << endl;
		timer++;
	}
	simulador->stopSimulation();
	sleep(2);

	delete simulador;
	delete argoCenter;

	for(int i = 0; i < numMotores; i++)
	{
		delete jointVector.at(i);
	}


}