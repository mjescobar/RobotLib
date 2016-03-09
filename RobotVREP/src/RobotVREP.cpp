
#include "RobotVREP.hpp"


void RobotVREP::trackConnection()
{
	int aux_clientID = simxGetConnectionId(clientID);

	if (aux_clientID == -1)
	{
		clog << "ERROR: The client is not connected to VREP. The program ends" << endl;
		delete this;
		exit(EXIT_FAILURE);
	}
	// else if (aux_clientID != clientID)
	// 	clog << "WARNING: Exist temporary disconections in-between" << endl;
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

RobotVREP::RobotVREP(bool video_recording, int port, const char * ip)
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

	if (simulation_in_progress) 
	{	
		stopSimulation();
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

int RobotVREP::getObjectHandle(char name[], simxInt operationMode)
{
	int handle;

	int error = simxGetObjectHandle(clientID, name, &handle, operationMode);	
	if(error != 0) vrep_error << "simxGetObjectHandle - " << name << " : "<< error << endl;

	return handle;
}

int RobotVREP::getCollisionHandle(char name[], simxInt operationMode)
{
	int collisionHandle;

	int error = simxGetCollisionHandle(clientID, name, &collisionHandle, operationMode);
	if(error != 0) vrep_error << "simxGetCollisionHandle: " << name << " : " << error << endl;

	return collisionHandle;
}

vector < double > RobotVREP::getObjectPosition(Object * object, int relativeTo, simxInt operationMode)
{
	int uniqueId = object->getUniqueObjectId();
	int vrepHandlerVectorPosition = objectIdToVrepHandler_map.at(uniqueId);
	int * handler = VrepHandlerVector.at(vrepHandlerVectorPosition);
	float * pos = new float[3];

	int error = simxGetObjectPosition(clientID, *handler, relativeTo, pos, operationMode);
	if(error != 0) vrep_error << "simxGetObjectPosition - " << *handler << " : "<< error << endl;

	vector < double > position;

	position.push_back((double)pos[0]);
	position.push_back((double)pos[1]);
	position.push_back((double)pos[2]);

	delete[] pos;

	return position;
}

vector < double > RobotVREP::getObjectVelocity(Object * object, simxInt operationMode)
{
	int uniqueId = object->getUniqueObjectId();
	int vrepHandlerVectorPosition = objectIdToVrepHandler_map.at(uniqueId);
	int * handler = VrepHandlerVector.at(vrepHandlerVectorPosition);
	float * lVel = new float[3];
	float * aVel = new float[3];

	int error = simxGetObjectVelocity(clientID, *handler, lVel, aVel, operationMode);
	if(error != 0) vrep_error << "simxGetObjectVelocity - " << *handler << " : "<< error << endl;

	vector < double > velocity;

	velocity.push_back((double)lVel[0]);
	velocity.push_back((double)lVel[1]);
	velocity.push_back((double)lVel[2]);
	velocity.push_back((double)aVel[0]);
	velocity.push_back((double)aVel[1]);
	velocity.push_back((double)aVel[2]);

	delete[] lVel;
	delete[] aVel;

	return velocity;
}

vector < double > RobotVREP::getObjectOrientation(Object * object, int relativeTo, simxInt operationMode)
{
	int uniqueId = object->getUniqueObjectId();
	int vrepHandlerVectorPosition = objectIdToVrepHandler_map.at(uniqueId);
	int * handler = VrepHandlerVector.at(vrepHandlerVectorPosition);
	float * ori = new float[3];

	int error = simxGetObjectOrientation(clientID,*handler, relativeTo, ori, operationMode);
	if(error != 0) vrep_error << "simxGetObjectOrientation - " << *handler << " : "<< error << endl;

	vector < double > orientation;

	orientation.push_back((double)ori[0]);
	orientation.push_back((double)ori[1]);
	orientation.push_back((double)ori[2]);

	delete[] ori;

	return orientation;
}

double RobotVREP::getJointPosition(Joint * joint, simxInt operationMode)
{
	int uniqueID = joint->getUniqueObjectId();
	int vrepHandlerVectorPosition =jointIdToVrepHandler_map.at ( uniqueID );
	int * handler = VrepHandlerVector.at( vrepHandlerVectorPosition );

	float joint_pos;

	int error = simxGetJointPosition(clientID, *handler, &joint_pos, operationMode);
	if(error != 0) vrep_error << "simxGetJointPosition - " << *handler << " : "<< error << endl;

	return (double)joint_pos;
}

void RobotVREP::setJointTargetPosition(Joint * joint, double joint_pos, simxInt operationMode)
{
	int uniqueID = joint->getUniqueObjectId();
	int vrepHandlerVectorPosition =jointIdToVrepHandler_map.at ( uniqueID );
	int * handler = VrepHandlerVector.at( vrepHandlerVectorPosition );

	int error = simxSetJointTargetPosition(clientID, *handler, (float)joint_pos, operationMode);
	if(error != 0) vrep_error << "simxSetJointTargetPosition - " << *handler << " : "<< error << endl;	
}

double RobotVREP::getJointForce(Joint * joint, simxInt operationMode)
{
	int uniqueID = joint->getUniqueObjectId();
	int vrepHandlerVectorPosition =jointIdToVrepHandler_map.at ( uniqueID );
	int * handler = VrepHandlerVector.at( vrepHandlerVectorPosition );

	float force;

	int error = simxGetJointForce(clientID, *handler, &force, operationMode);
	if(error != 0) vrep_error << "simxGetJointForce - " << *handler << " : "<< error << endl;

	return (double)force;
}

void RobotVREP::addStatusbarMessage(char * message, simxInt operationMode)
{
	int error = simxAddStatusbarMessage(clientID,(char*)message, operationMode);	
	if(error != 0) vrep_error << "simxAddStatusbarMessage - " << message << " : " << error << endl;
} 

bool RobotVREP::readCollision(CollisionObject * collisionObject, simxInt operationMode)
{
	int uniqueID = collisionObject->getUniqueObjectId();
	int vrepHandlerVectorPosition =collisionObjectIdToVrepHandler_map.at ( uniqueID );
	int * handler = VrepHandlerVector.at( vrepHandlerVectorPosition );

	unsigned char * aux = new unsigned char[100];

	int error = simxReadCollision(clientID, *handler, aux, operationMode);
	if(error != 0) vrep_error << "simxReadCollision - " << *handler << " : " << error << endl;	

	int collisionState = (int)*aux;
	bool boolCollisionState = (collisionState == 0) ? false : true;
	delete aux;

	collisionObject->setCollisionState(boolCollisionState);

	return boolCollisionState;
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


void RobotVREP::addJoint( Joint * joint)
{
	int * handler = new int(getObjectHandle(joint->getName(), simx_opmode_oneshot_wait));	
	VrepHandlerVector.push_back( handler );
	jointVector.push_back( joint );
	jointIdToVrepHandler_map.insert( std::pair<int,int> ( joint->getUniqueObjectId() , VrepHandlerVector.size() -1 ) );

	getJointForce(joint, simx_opmode_streaming);
	getJointPosition(joint, simx_opmode_streaming);
}

void RobotVREP::addObject( Object * object)
{
	int * handler = new int(getObjectHandle(object->getName(), simx_opmode_oneshot_wait));
	VrepHandlerVector.push_back( handler );
	objectVector.push_back( object );
	objectIdToVrepHandler_map.insert( std::pair<int,int> ( object->getUniqueObjectId() , VrepHandlerVector.size() -1 ) );

	getObjectPosition(object, -1, simx_opmode_streaming);
	getObjectOrientation(object, -1, simx_opmode_streaming);
	getObjectVelocity(object, simx_opmode_streaming);
}

void RobotVREP::addCollisionObject( CollisionObject * collisionObject)
{
	int * handler = new int(getObjectHandle(collisionObject->getName(), simx_opmode_oneshot_wait));
	VrepHandlerVector.push_back( handler );
	collisionObjectVector.push_back( collisionObject );
	collisionObjectIdToVrepHandler_map.insert( std::pair<int,int> ( collisionObject->getUniqueObjectId() , VrepHandlerVector.size() -1 ) );

	readCollision(collisionObject, simx_opmode_streaming);
}

void RobotVREP::moveJoints()
{
	pauseCommunication(1);
	trackConnection();

	for (unsigned int i = 0; i < jointVector.size(); ++i)
	{
		double position = jointVector.at(i)->getNextPositionRad(); 
		setJointTargetPosition( jointVector.at(i), position);
	}
	pauseCommunication(0);
}

void RobotVREP::moveJointsToInitialPosition()
{
	pauseCommunication(1);
	trackConnection();

	for (unsigned int i = 0; i < jointVector.size(); ++i)
	{
		double position = jointVector.at(i)->getJointInitialPositionRad(); 
		setJointTargetPosition( jointVector.at(i), position);
	}
	pauseCommunication(0);
}

bool RobotVREP::checkAllCollisions()
{
	trackConnection();

	for (unsigned int i = 0; i < collisionObjectVector.size(); ++i)
	{
		if(readCollision(collisionObjectVector.at(i)))
			return true;
	}

	return false;
}
