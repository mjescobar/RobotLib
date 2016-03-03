#include "Joint.hpp"
#include "robotObject.hpp"
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
		Joint * joint  = new Joint(M_PI/4.0, -M_PI/4.0, (char *)"SCALE" );
		jointVector.push_back( joint );
		stringstream jointName;
		jointName << "joint" << i << "#";
		simulador->addMotor( joint, (char *)jointName.str().c_str());
	}


	// Agregando los sensores

	RobotObject * argoCenter = new RobotObject();

	simulador->addSensor(argoCenter, (char *) "ArgoV2" );


	simulador->startSimulation();
	while( true )
	{
		for (unsigned int i = 0; i < jointVector.size(); ++i)
		{
			double delta = (2*(rand()/(double)(RAND_MAX)) -1 )*0.1; 
			jointVector.at(i)->setJointNextPosition( jointVector.at(i)->getJointGoalPosition() + delta );
		}
		simulador->move();
		std::vector <float> position;
		position = simulador->getWorldPosition(argoCenter);
		cout << "Argo position: " << position.at(0) << ", " << position.at(1) << ", " << position.at(2) << endl;
	}

}