/*
 * Mission.h
 *
 *  Created on: 15.03.2018
 *      Author: mremm
 */

#ifndef MISSION_H_
#define MISSION_H_

#include "Command.h"

template<typename E>
struct LinkedElement {
    int index;
    E element;
    LinkedElement* next;
    LinkedElement* prev;
};

class Mission {
    int size = 0;
    bool finalized = false;
protected:
    LinkedElement<Command*>* first;
    LinkedElement<Command*>* dhis;
public:
    Mission();
    virtual ~Mission();

    void add(Command* command);

    bool empty()
    {
        return (dhis == nullptr && first == nullptr);
    }

    Mission& finalize();

    Mission& start()
    {
        dhis = first;
        return *this;
    }

    Command* next()
    {
        dhis = dhis->next;
        return dhis->element;
    }

    Command* prev()
    {
        dhis = dhis->prev;
        return dhis->element;
    }
    int index()
    {
        return dhis->index;
    }

    int getCommandIndex()
    {
        return index();
    }

    Command* getNextCommand()
    {
        return dhis->next->element;
    }
    Command* getCurrCommand()
    {
        return dhis->element;
    }
    Command* getPrevCommand()
    {
        return dhis->prev->element;
    }
};

#endif /* MISSION_H_ */
