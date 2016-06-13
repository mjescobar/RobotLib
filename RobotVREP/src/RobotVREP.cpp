
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

int RobotVREP::getObjectHandler(Object * object)
{
	int uniqueId = object->getUniqueObjectId();
	int vrepHandlerVectorPosition = objectIdToVrepHandler_map.at(uniqueId);
	int * handler = VrepHandlerVector.at(vrepHandlerVectorPosition);

	return *handler;
}

void RobotVREP::setVideoRecordingMode(bool active)
{
	int error;	

	if(active)
	{		
		char * aux_old_path;

		error = simxGetStringParameter(clientID, sim_stringparam_video_filename, (simxChar**)&aux_old_path,simx_opmode_oneshot_wait);
		if(error != 0) vrep_error << "simxGetStringParameter: " << error << endl;		

		old_path = string(aux_old_path);

		char * response;
		int responselen;

		error = simxGetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar**)&response, (simxInt*)&responselen,simx_opmode_streaming);
		if(error != 0) vrep_error << "simxGetStringSignal: videoPath - " << error << endl;

		video_recording_flag = true;

		// Change video recording path folder

		char cwd[1024];	    

	    if (getcwd(cwd, sizeof(cwd)) != NULL)
	    {
	    	int init_space = 0;
	    	for (int i = 0; i < (int)strlen(cwd); i++)
	    		if(cwd[i] == ' ') init_space++;
	    		
	        char cwd_aux[strlen(cwd) + init_space];
	        int space = 0;

	        for(int i = 0; i < (int)strlen(cwd) + 1; i++)
	        {   
	            if(cwd[i] == ' ')
	            {                
	                cwd_aux[i+space] = 0x5C;
	                space++;
	            }

	            cwd_aux[i+space] = cwd[i];
	        }

	        stringstream videoDirectory, videoDirectory_original;	        

	        videoDirectory_original << cwd << "/videoRecorder";
	        new_path = string(videoDirectory_original.str().c_str());

	        videoDirectory << cwd_aux << "/videoRecorder";
	        string s = videoDirectory.str(); 

	        clog << "ROBOTLIB:\tChanging video path to:\t" << s << endl;

	        stringstream mkdir, rm;
	        mkdir << "mkdir -p " << s;
	        rm << "rm -f " << s << "/*";

	        system((char *)mkdir.str().c_str());
	        system((char *)rm.str().c_str());

	        changeVideoPath((char *)videoDirectory_original.str().c_str());
	    }
	    else
	        perror("getcwd() error");
		
	}
	else if(video_recording_flag)
	{
		int error = simxSetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar*)old_path.c_str(), (simxInt)(old_path.size()), simx_opmode_oneshot);
		if(error != 0) vrep_error << "simxSetStringSignal: videoPath - " << error << endl;

		video_recording_flag = false;
	}
	
	error = simxSetBooleanParameter(clientID, sim_boolparam_browser_visible,false, simx_opmode_oneshot);
	if(error != 0) vrep_error << " simxSetBooleanParameter : sim_boolparam_browser_visible - " << error << endl;
	error = simxSetBooleanParameter(clientID, sim_boolparam_hierarchy_visible,false, simx_opmode_oneshot);
	if(error != 0) vrep_error << " simxSetBooleanParameter : sim_boolparam_hierarchy_visible - " << error << endl;

}

RobotVREP::RobotVREP(bool video_recording, int port, const char * ip)
{
	clientID = simxStart((simxChar*)ip, port, true, true, 2000, 5);
	if (clientID != -1) clog << "ROBOTLIB:\tThe connection has been successfully established with VREP" << endl;
	else 
	{
		clog << "ROBOTLIB:" << endl;
		clog << "\tERROR: The connection to VREP was not possible" << endl;
		return;
	}

	vrep_error.open("vrep_error.txt");

	simulation_in_progress = false;
	video_recording_flag = false;
	video_updated_name = false;
	video_id = 0;

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

int RobotVREP::getLastCmdTime()
{
	return simxGetLastCmdTime(clientID);
}

void RobotVREP::startSimulation(simxInt operationMode)
{
	int error;

	trackConnection();	

	if(video_recording_flag)
	{
		if(!video_updated_name)
		{
			stringstream video_name;
			video_name << "videoRecorder" << video_id;
			changeVideoName((char*)video_name.str().c_str(), simx_opmode_oneshot_wait);
			video_id++;
		}
		
		error = simxSetBooleanParameter(clientID, sim_boolparam_video_recording_triggered, 1, simx_opmode_oneshot);
		if(error != 0) vrep_error << " simxSetBooleanParameter : sim_boolparam_video_recording_triggered - " << error << endl;
	}

	error = simxStartSimulation(clientID, operationMode);
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

	video_updated_name = false;

	sleep(1);
}

void RobotVREP::pauseSimulation(simxInt operationMode)
{
	int error = simxPauseSimulation(clientID, operationMode);
	if(error != 0) vrep_error << "simxPauseSimulation : " << error << endl;
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

void RobotVREP::setObjectPosition(Object * object, vector < double > position, int relativeTo, simxInt operationMode)
{
	int uniqueId = object->getUniqueObjectId();
	int vrepHandlerVectorPosition = objectIdToVrepHandler_map.at(uniqueId);
	int * handler = VrepHandlerVector.at(vrepHandlerVectorPosition);
	float * pos = new float[3];

	pos[0] = (float)position.at(0);
	pos[1] = (float)position.at(1);
	pos[2] = (float)position.at(2);

	int error = simxSetObjectPosition(clientID, *handler, relativeTo, pos, operationMode);
	if(error != 0) vrep_error << "simxSetObjectPosition: " << *handler << " : " << error << endl;

	delete[] pos;
}

void RobotVREP::setObjectOrientation(Object * object, vector < double > orientation, int relativeTo, simxInt operationMode)
{
	int uniqueId = object->getUniqueObjectId();
	int vrepHandlerVectorPosition = objectIdToVrepHandler_map.at(uniqueId);
	int * handler = VrepHandlerVector.at(vrepHandlerVectorPosition);
	float * ori = new float[3];

	ori[0] = (float)orientation.at(0);
	ori[1] = (float)orientation.at(1);
	ori[2] = (float)orientation.at(2);

	int error = simxSetObjectOrientation(clientID, *handler, relativeTo, ori, operationMode);
	if(error != 0) vrep_error << "simxSetObjectOrientation: " << *handler << " : " << error << endl;

	delete[] ori;
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

void RobotVREP::setJointTargetVelocity(Joint * joint, double joint_vel, simxInt operationMode)
{
	int uniqueID = joint->getUniqueObjectId();
	int vrepHandlerVectorPosition =jointIdToVrepHandler_map.at ( uniqueID );
	int * handler = VrepHandlerVector.at( vrepHandlerVectorPosition );

	int error = simxSetJointTargetVelocity(clientID, *handler, (float)joint_vel, operationMode);
    if(error != 0) vrep_error << "simxSetJointTargetVelocity: " << *handler << " : " << error << endl;
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
	delete[] aux;

	collisionObject->setCollisionState(boolCollisionState);

	return boolCollisionState;
}

vector < int > RobotVREP::getVisionSensorResolution(VisionSensor * visionSensor, simxInt operationMode)
{
	int uniqueID = visionSensor->getUniqueObjectId();
	int vrepHandlerVectorPosition = visionSensorIdToVrepHandler_map.at ( uniqueID );
	int * handler = VrepHandlerVector.at( vrepHandlerVectorPosition );

	int resolution_aux[2];

	int error = simxGetVisionSensorImage(clientID, *handler, resolution_aux, NULL, 0, operationMode);
    if(error != 0) vrep_error << "simxGetVisionSensorImage: " << *handler << " : " << error << endl;

 	vector < int > resolution;
 	resolution.push_back(resolution_aux[0]);
 	resolution.push_back(resolution_aux[1]);
 	
 	visionSensor->resolution = resolution;

 	return resolution;
}

unsigned char * RobotVREP::getVisionSensorImage(VisionSensor * visionSensor, simxInt operationMode)
{
	int uniqueID = visionSensor->getUniqueObjectId();
	int vrepHandlerVectorPosition = visionSensorIdToVrepHandler_map.at ( uniqueID );
	int * handler = VrepHandlerVector.at( vrepHandlerVectorPosition );
	
	unsigned char * image;
	int resolution_aux[2];
	
	int error = simxGetVisionSensorImage(clientID, *handler, resolution_aux, &image, 0, operationMode);

	vector < int > resolution;
 	resolution.push_back(resolution_aux[0]);
 	resolution.push_back(resolution_aux[1]);

	visionSensor->resolution = resolution;

    if(error != 0) vrep_error << "simxGetVisionSensorImage: " << *handler << " : " << error << endl;

    return image;
}

void RobotVREP::changeVideoPath(char path[], simxInt operationMode)
{	
	if(!simulation_in_progress)
	{
		int pathlen = strlen(path)+1;

		int error = simxSetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar*)path, (simxInt)pathlen, operationMode);
		if(error != 0) vrep_error << "simxSetStringSignal: videoPath - " << error << endl;

		char * response;
		int responselen;

		do
		{	
			usleep(100000);

			int error = simxGetStringSignal(clientID, (simxChar*)"videoPath", (simxUChar**)&response, (simxInt*)&responselen,simx_opmode_buffer);
			if(error != 0) vrep_error << "simxGetStringSignal: videoPath - " << error << endl;
		}
		while(responselen == pathlen);

		if (!video_recording_flag)
		{
			clog << "ROBOTLIB:" << endl;
		 	clog << "\tWARNING: The video recording mode is not active." << endl;
		}
	}
	else
	{
		clog << "ROBOTLIB:" << endl;
		clog << "\tERROR: This function can only be used if the simulator is stopped." << endl;
	}
}

void RobotVREP::changeVideoName(char name[], simxInt operationMode)
{
	stringstream new_full_path;
	new_full_path << new_path << "/" << name;

	clog << "ROBOTLIB:\tChanging video name to:\t" << name << endl;

	changeVideoPath((char *)new_full_path.str().c_str(), operationMode);

	video_updated_name = true;
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

void RobotVREP::addVisionSensor( VisionSensor * visionSensor )
{
	int * handler = new int(getObjectHandle(visionSensor->getName(), simx_opmode_oneshot_wait));
	VrepHandlerVector.push_back( handler );
	visionSensorVector.push_back( visionSensor );
	visionSensorIdToVrepHandler_map.insert( std::pair<int,int> ( visionSensor->getUniqueObjectId() , VrepHandlerVector.size() -1 ) );
	
	getVisionSensorResolution(visionSensor, simx_opmode_streaming);
		
	addObject(visionSensor);	
}

void RobotVREP::addCollisionObject( CollisionObject * collisionObject)
{
	int * handler = new int(getCollisionHandle(collisionObject->getName(), simx_opmode_oneshot_wait));
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
