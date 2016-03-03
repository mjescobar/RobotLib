
#include "RobotVREP.hpp"
#include "Joint.hpp"


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

void RobotVREP::setVideoRecordingMode(bool active)
{
	int error;	

	if(active)
	{		
		char * aux_old_path;
		int aux_old_path_len;

		error = simxGetStringParameter(clientID, sim_stringparam_video_filename, (simxChar**)&aux_old_path,simx_opmode_oneshot_wait);
		if(error != 0) vrep_error << "simxGetStringParameter: " << error << endl;		

		aux_old_path_len = strlen(aux_old_path)+1;
		old_path = new char[aux_old_path_len];
		strncpy(old_path, aux_old_path, aux_old_path_len);

		char * response;
		int responselen;

		error = simxGetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar**)&response, (simxInt*)&responselen,simx_opmode_streaming);
		if(error != 0) vrep_error << "simxGetStringSignal: videoPath - " << error << endl;

		video_recording_flag = active;
	}
	else if(video_recording_flag)
	{
		int error = simxSetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar*)old_path, (simxInt)(strlen(old_path)+1), simx_opmode_oneshot);
		if(error != 0) vrep_error << "simxSetStringSignal: videoPath - " << error << endl;

		video_recording_flag = false;
	}

	error = simxSetBooleanParameter(clientID, sim_boolparam_video_recording_triggered, active, simx_opmode_oneshot);
	if(error != 0) vrep_error << " simxSetBooleanParameter : sim_boolparam_video_recording_triggered" << error << endl;
	error = simxSetBooleanParameter(clientID, sim_boolparam_browser_visible,!active, simx_opmode_oneshot);
	if(error != 0) vrep_error << " simxSetBooleanParameter : sim_boolparam_browser_visible" << error << endl;
	error = simxSetBooleanParameter(clientID, sim_boolparam_hierarchy_visible,!active, simx_opmode_oneshot);
	if(error != 0) vrep_error << " simxSetBooleanParameter : sim_boolparam_hierarchy_visible" << error << endl;

}

RobotVREP::RobotVREP(bool video_recording)
{
	clientID = simxStart((simxChar*)"127.0.0.1",PORTNB,true,true,2000,5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");

	simulation_in_progress = false;

	setVideoRecordingMode(video_recording);
}

RobotVREP::RobotVREP(const char * ip, bool video_recording)
{
	clientID = simxStart((simxChar*)ip, PORTNB, true, true, 2000, 5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else 
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");

	simulation_in_progress = false;

	setVideoRecordingMode(video_recording);
}

RobotVREP::RobotVREP(int port, bool video_recording)
{
	clientID = simxStart((simxChar*)"127.0.0.1", port, true, true, 2000, 5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else 
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");

	simulation_in_progress = false;

	setVideoRecordingMode(video_recording);
}

RobotVREP::RobotVREP(const char * ip, int port, bool video_recording)
{
	clientID = simxStart((simxChar*)ip, port, true, true, 2000, 5);
	if (clientID != -1) clog << "The connection has been successfully established with VREP" << endl;
	else 
	{
		clog << "ERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("error_files/vrep_error.txt");

	simulation_in_progress = false;

	setVideoRecordingMode(video_recording);
}

RobotVREP::~RobotVREP()
{
	if (video_recording_flag) 
	{
		setVideoRecordingMode(false);
	}

	simxFinish(clientID);

	vrep_error.close();
}

int RobotVREP::getConnectionId()
{
	return simxGetConnectionId(clientID);
}

void RobotVREP::pauseCommunication(int action)
{
	simxPauseCommunication(clientID, action);
}

void RobotVREP::startSimulation(simxInt operationMode)
{
	trackConnection();

	int error = simxStartSimulation(clientID, operationMode);
	if(error != 0) vrep_error << " try 1 simxStartSimulation : " << error << endl;
	else
	{
		error = simxStartSimulation(clientID, operationMode);	
		if(error != 0) vrep_error << "try 2 simxStartSimulation : " << error << endl;
	} 

	if(error == 0) simulation_in_progress = true;	

	usleep(100000);
}

void RobotVREP::stopSimulation(simxInt operationMode)
{
	int error = simxStopSimulation(clientID, operationMode);
	if(error != 0) vrep_error << "simxStopSimulation : " << error << endl;

	if(error == 0) simulation_in_progress = false;

	usleep(100000);
}

void RobotVREP::getObjectHandle(char name[], int * handle, simxInt operationMode)
{
	int error = simxGetObjectHandle(clientID, name, handle, operationMode);	
	if(error != 0) vrep_error << "simxGetObjectHandle - " << name << " : "<< error << endl;
}

void RobotVREP::getObjectPosition(int object_handle, int relativeTo, double * position, simxInt operationMode)
{
	float * aux = new float[3];

	int error = simxGetObjectPosition(clientID, object_handle, relativeTo, aux, operationMode);
	if(error != 0) vrep_error << "simxGetObjectPosition - " << object_handle << " : "<< error << endl;

	for(int i = 0; i < 3; i++)
		position[i] = (double)aux[i];
}

void RobotVREP::getObjectVelocity(int object_handle, double * lVelocity, double * aVelocity, simxInt operationMode)
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

void RobotVREP::getObjectOrientation(int object_handle, int relativeTo, double * orientation, simxInt operationMode)
{
	float * aux = new float[3];

	int error = simxGetObjectOrientation(clientID,object_handle, relativeTo, aux, operationMode);
	if(error != 0) vrep_error << "simxGetObjectOrientation - " << object_handle << " : "<< error << endl;

	for(int i = 0; i < 3; i++)
		orientation[i] = (double)aux[i];
}

double RobotVREP::getJointPosition(int object_handle, simxInt operationMode)
{
	float joint_pos;

	int error = simxGetJointPosition(clientID, object_handle, &joint_pos, operationMode);
	if(error != 0) vrep_error << "simxGetJointPosition - " << object_handle << " : "<< error << endl;

	return (double)joint_pos;
}

void RobotVREP::setJointTargetPosition(int object_handle, double joint_pos, simxInt operationMode)
{
	int error = simxSetJointTargetPosition(clientID, object_handle, (float)joint_pos, operationMode);
	if(error != 0) vrep_error << "simxSetJointTargetPosition - " << object_handle << " : "<< error << endl;	
}

double RobotVREP::getJointForce(int object_handle, simxInt operationMode)
{
	float force;

	int error = simxGetJointForce(clientID, object_handle, &force, operationMode);
	if(error != 0) vrep_error << "simxGetJointForce - " << object_handle << " : "<< error << endl;

	return (double)force;
}

void RobotVREP::addStatusbarMessage(char * message, simxInt operationMode)
{
	int error = simxAddStatusbarMessage(clientID,(char*)message, operationMode);	
	if(error != 0) vrep_error << "simxAddStatusbarMessage - " << message << " : " << error << endl;
} 

void RobotVREP::readCollision(int collisionHandle, int * collisionState, simxInt operationMode)
{
	unsigned char * aux = (unsigned char*)malloc(sizeof(unsigned char)*100);
	int error = simxReadCollision(clientID, collisionHandle, aux, operationMode);
	if(error != 0) vrep_error << "simxReadCollision - " << collisionHandle << " : " << error << endl;	
	*collisionState = (int)*aux;
	free(aux);
}

void RobotVREP::getCollisionHandle(char name[], int * collisionHandle, simxInt operationMode)
{
	int error = simxGetCollisionHandle(clientID, name, collisionHandle, operationMode);
	if(error != 0) vrep_error << "simxGetCollisionHandle: " << name << " : " << error << endl;
}


void RobotVREP::changeVideoPath(char path[], simxInt operationMode)
{	
	if(!simulation_in_progress)
	{
		int pathlen = strlen(path)+1;

		int error = simxSetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar*)path, (simxInt)pathlen, operationMode);
		if(error != 0) 
		{
			vrep_error << "simxSetStringSignal: videoPath - " << error << endl;
		}

		char * response;
		int responselen;

		do
		{	
			usleep(100000);

			int error = simxGetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar**)&response, (simxInt*)&responselen,simx_opmode_buffer);
			if(error != 0) 
			{
				vrep_error << "simxGetStringSignal: videoPath - " << error << endl;
			}
		}
		while(responselen == pathlen);

		if (!video_recording_flag)
		{
		 	clog << "WARNING: The video recording mode is not active." << endl;
		}
	}
	else
	{
		clog << "ERROR: This function can only be used if the simulator is stopped." << endl;
	}
}


void RobotVREP::addMotor( Joint * joint, char name[] )
{
	int * handler = new int();
	getObjectHandle(name, handler, simx_opmode_oneshot_wait);
	VrepHandlerVector.push_back( handler );
	jointVector.push_back( joint );
	jointIdToVrepHandler_map.insert( std::pair<int,int> ( joint->getUniqueRobotObjectId() , VrepHandlerVector.size() -1 ) );
}

void RobotVREP::addSensor( RobotObject * robotObject, char name[] )
{
	int * handler = new int();
	getObjectHandle(name, handler, simx_opmode_oneshot_wait);
	VrepHandlerVector.push_back( handler );
	sensorVector.push_back( robotObject );
	sensorIdToVrepHandler_map.insert( std::pair<int,int> ( robotObject->getUniqueRobotObjectId() , VrepHandlerVector.size() -1 ) );
}

void RobotVREP::move()
{
	pauseCommunication(1);
	if ( getConnectionId() == -1 )
	{
		cerr << "The connection with VREP fail" << endl;
		exit(EXIT_FAILURE);
	}

	for (unsigned int i = 0; i < jointVector.size(); ++i)
	{
		int jointId = jointVector.at(i)->getUniqueRobotObjectId();
		int * handler =  VrepHandlerVector.at( jointIdToVrepHandler_map.at( jointId ) );
		double position = jointVector.at(i)->getNextPositionRad();
		setJointTargetPosition( *handler, position, simx_opmode_oneshot);
	}
	pauseCommunication(0);
}

vector <float> RobotVREP::getWorldPosition(RobotObject * robotObject)
{
	float * aux = new float[3];

	int vrepHandlerPosition = sensorIdToVrepHandler_map.at( robotObject->getUniqueRobotObjectId() );
	int * handler = VrepHandlerVector.at(vrepHandlerPosition);

	int error = simxGetObjectPosition(clientID, *handler, -1, aux, simx_opmode_oneshot_wait);
	if(error != 0) vrep_error << "simxGetObjectPosition - " << *handler << " : "<< error << endl;

	vector <float> result;
	result.push_back(aux[0]);
	result.push_back(aux[1]);
	result.push_back(aux[2]);
	delete[] aux;
	return result;
}