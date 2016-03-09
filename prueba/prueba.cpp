#include "Joint.hpp"
#include "RobotVREP.hpp"
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
	RobotVREP * simulador = new RobotVREP();


	// Agregando los motores.
	for(int i = 0; i < numMotores; i++)
	{
		stringstream jointName;
		jointName << "joint" << i << "#";
		Joint * joint  = new Joint((char *)jointName.str().c_str(), M_PI/4.0, -M_PI/4.0, 0.0,(char *)"SCALE" );
		jointVector.push_back( joint );		
		simulador->addJoint( joint );
	}

	// Agregando los sensores

	Object * argoCenter = new Object((char *)"ArgoV2_reference");

	simulador->addObject(argoCenter);


	simulador->startSimulation();
	while( true )
	{
		for (unsigned int i = 0; i < jointVector.size(); ++i)
		{
			double delta = (2*(rand()/(double)(RAND_MAX)) -1 )*0.1; 
			jointVector.at(i)->setJointNextPosition( jointVector.at(i)->getJointGoalPosition() + delta );
		}
		simulador->moveJoints();
		std::vector <double> position;
		position = simulador->getObjectPosition(argoCenter);
		cout << "Argo position: " << position.at(0) << ", " << position.at(1) << ", " << position.at(2) << endl;
	}

	delete simulador;
	delete argoCenter;

	for(int i = 0; i < numMotores; i++)
	{
		delete jointVector.at(i);
	}


}