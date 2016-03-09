#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string.h>

class Object
{
	int uniqueObjectId;
	// Name of a object, needed for use an virtual or real environment workspace.
	char * name;
public:
	static int uniqueObjectIdGenerator;
	int getUniqueObjectId();

	/**
	 * \brief Constructor with parameters.
	 * \param name Name of the object in VREP.
	 */	
	Object( const char name[] = "unknown");

	/**
	 * \brief Retrieves the name corresponding to object.
	 * \return The object name.
	 */
	char * getName();
};

#endif