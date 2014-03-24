#include "manController.h"
/************************************************************************/
/*  W/UpArrow:GoForward                                                 */
/*  S/DownArrow:GoBackward                                              */
/*  A/LeftArrow:TurnLeft                                                */
/*  D/RightArrow:TurnRight                                              */
/*  J        :Jump                                                      */
/*  P        :Pause                                                     */
/************************************************************************/
manController::manController(Character* man)
{
	m_man = man;
	m_ghostObject = m_man->getGhostObject();
}
manController::~manController(void)
{
}

bool manController::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	switch (ea.getEventType())
	{
	case osgGA::GUIEventAdapter::KEYDOWN:
		switch(ea.getKey())
		{
		case 'w':
		case 'W':
		case osgGA::GUIEventAdapter::KEY_Up:
			m_man->goForward();
			cout<<"Forward;"<<endl;
			break;
		case 's':
		case 'S':
		case osgGA::GUIEventAdapter::KEY_Down:
			m_man->goBackward();
			cout<<"Backward;"<<endl;
			break;
		case 'a':
		case 'A':
		case osgGA::GUIEventAdapter::KEY_Left:
			m_man->goLeft();
			cout<<"Left;"<<endl;
			break;
		case 'd':
		case 'D':
		case osgGA::GUIEventAdapter::KEY_Right:
			m_man->goRight();
			cout<<"Right;"<<endl;
			break;
		case osgGA::GUIEventAdapter::KEY_J:
			m_man->goJump();
			break;
		case osgGA::GUIEventAdapter::KEY_P:
			m_man->idle();
			break;
		default:
			break;
		}
		return true;
		break;
	default:
		break;
	}
	return false;
}
