#include "Object.hpp"

int Object::uniqueObjectIdGenerator = 1;
Object::Object( const char name[] )
{
	uniqueObjectId = uniqueObjectIdGenerator++;

	int size = strlen(name)+1;
	this->name = new char[size];
	strncpy(this->name, name, size);
}

int Object::getUniqueObjectId()
{
	return uniqueObjectId;
}

char * Object::getName()
{
	return name;
}