#pragma once
#include "Character.h"
#include <osgGA/GUIEventHandler>
#include <iostream>
using namespace std;

class manController :
	public osgGA::GUIEventHandler 
{
public:
	manController(Character* man);
	~manController(void);
	virtual bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&);

private:
	
	class Character*		m_man;
	class btGhostObject*	m_ghostObject;
};

