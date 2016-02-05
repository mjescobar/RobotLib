#ifndef ROBOTVREP_CPP
#define ROBOTVREP_CPP

#include "RobotVREP.hpp"

using namespace ANN_USM;

RobotVREP::RobotVREP()
{
	clientID = simxStart((simxChar*)"127.0.0.1",PORTNB,true,true,2000,5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");
}

RobotVREP::RobotVREP(const char * ip)
{
	clientID = simxStart((simxChar*)ip, PORTNB, true, true, 2000, 5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else 
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");
}

RobotVREP::RobotVREP(int port)
{
	clientID = simxStart((simxChar*)"127.0.0.1", port, true, true, 2000, 5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else 
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");
}

RobotVREP::RobotVREP(const char * ip, int port)
{
	clientID = simxStart((simxChar*)ip, port, true, true, 2000, 5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else 
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");
}

RobotVREP::~RobotVREP()
{
	simxFinish(clientID);

	vrep_error.close();
}

void RobotVREP::trackConnection()
{
	int aux_clientID = simxGetConnectionId(clientID);

	if (aux_clientID == -1)
	{
		clog << "ERROR: The client is not connected to VREP. The program ends" << endl;
		delete this;
		exit(EXIT_FAILURE);
	}
	else if (aux_clientID != clientID)
		clog << "WARNING: Exist temporary disconections in-between" << endl;
}

int RobotVREP::GetConnectionId()
{
	return simxGetConnectionId(clientID);
}

void RobotVREP::PauseCommunication(int action)
{
	simxPauseCommunication(clientID, action);
}

void RobotVREP::StartSimulation(simxInt operationMode)
{
	trackConnection();

	int error = simxStartSimulation(clientID, operationMode);
	if(error != 0) vrep_error << " try 1 simxStartSimulation : " << error << endl;
	else
	{
		error = simxStartSimulation(clientID, operationMode);	
		if(error != 0) vrep_error << "try 2 simxStartSimulation : " << error << endl;
	} 	

	usleep(100000);
}

void RobotVREP::StopSimulation(simxInt operationMode)
{
	int error = simxStopSimulation(clientID, operationMode);
	if(error != 0) vrep_error << "simxStopSimulation : " << error << endl;

	usleep(100000);
}

void RobotVREP::GetObjectHandle(char name[], int * handle, simxInt operationMode)
{
	int error = simxGetObjectHandle(clientID, name, handle, operationMode);	
	if(error != 0) vrep_error << "simxGetObjectHandle - " << name << " : "<< error << endl;
}

void RobotVREP::GetObjectPosition(int object_handle, int relativeTo, double * position, simxInt operationMode)
{
	float * aux = new float[3];

	int error = simxGetObjectPosition(clientID, object_handle, relativeTo, aux, operationMode);
	if(error != 0) vrep_error << "simxGetObjectPosition - " << object_handle << " : "<< error << endl;

	for(int i = 0; i < 3; i++)
		position[i] = (double)aux[i];
}

void RobotVREP::GetObjectVelocity(int object_handle, double * lVelocity, double * aVelocity, simxInt operationMode)
{
	float * lVel = new float[3];
	float * aVel = new float[3];

	int error = simxGetObjectVelocity(clientID, object_handle, lVel, aVel, operationMode);
	if(error != 0) vrep_error << "simxGetObjectVelocity - " << object_handle << " : "<< error << endl;

	if((lVelocity == NULL) && (aVelocity == NULL))
		clog << "ERROR: lVelocity and aVelocity can not be null at the same time" << endl;
	else
	{
		if(lVelocity != NULL)
		for(int i = 0; i < 3; i++)
			lVelocity[i] = (double)lVel[i];

	if(aVelocity != NULL)
		for(int i = 0; i < 3; i++)
			aVelocity[i] = (double)aVel[i];
	}	
}

void RobotVREP::GetObjectOrientation(int object_handle, int relativeTo, double * orientation, simxInt operationMode)
{
	float * aux = new float[3];

	int error = simxGetObjectOrientation(clientID,object_handle, relativeTo, aux, operationMode);
	if(error != 0) vrep_error << "simxGetObjectOrientation - " << object_handle << " : "<< error << endl;

	for(int i = 0; i < 3; i++)
		orientation[i] = (double)aux[i];
}

double RobotVREP::GetJointPosition(int object_handle, simxInt operationMode)
{
	float joint_pos;

	int error = simxGetJointPosition(clientID, object_handle, &joint_pos, operationMode);
	if(error != 0) vrep_error << "simxGetJointPosition - " << object_handle << " : "<< error << endl;

	return (double)joint_pos;
}

void RobotVREP::SetJointTargetPosition(int object_handle, double joint_pos, simxInt operationMode)
{
	int error = simxSetJointTargetPosition(clientID, object_handle, (float)joint_pos, operationMode);
	if(error != 0) vrep_error << "simxSetJointTargetPosition - " << object_handle << " : "<< error << endl;	
}

double RobotVREP::GetJointForce(int object_handle, simxInt operationMode)
{
	float force;

	int error = simxGetJointForce(clientID, object_handle, &force, operationMode);
	if(error != 0) vrep_error << "simxGetJointForce - " << object_handle << " : "<< error << endl;

	return (double)force;
}

void RobotVREP::AddStatusbarMessage(char * message, simxInt operationMode)
{
	int error = simxAddStatusbarMessage(clientID,(char*)message, operationMode);	
	if(error != 0) vrep_error << "simxAddStatusbarMessage - " << message << " : " << error << endl;
} 

void RobotVREP::ReadCollision(int collisionHandle, int * collisionState, simxInt operationMode)
{
	unsigned char * aux = (unsigned char*)malloc(sizeof(unsigned char)*100);
	int error = simxReadCollision(clientID, collisionHandle, aux, operationMode);
	if(error != 0) vrep_error << "simxReadCollision - " << collisionHandle << " : " << error << endl;	
	*collisionState = (int)*aux;
	free(aux);
}

void RobotVREP::GetCollisionHandle(char name[], int * collisionHandle, simxInt operationMode)
{
	int error = simxGetCollisionHandle(clientID, name, collisionHandle, operationMode);
	if(error != 0) vrep_error << "simxGetCollisionHandle: " << name << " : " << error << endl;
}

#endif