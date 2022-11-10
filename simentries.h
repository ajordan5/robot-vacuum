#ifndef SIMENTRIES_H
#define SIMENTRIES_H

#include "C:/bullet3/examples/CommonInterfaces/CommonExampleInterface.h"

class SimEntries : public ExampleEntries
{
    struct ExampleEntriesInternalData* m_data;

public:
    SimEntries();
    virtual ~SimEntries();

    static void registerExampleEntry(int menuLevel, const char* name, const char* description, CommonExampleInterface::CreateFunc* createFunc, int option = 0);

    virtual void initExampleEntries();

    virtual void initOpenCLExampleEntries();

    virtual int getNumRegisteredExamples();

    virtual CommonExampleInterface::CreateFunc* getExampleCreateFunc(int index);

    virtual const char* getExampleName(int index);

    virtual const char* getExampleDescription(int index);

    virtual int getExampleOption(int index);

};

#endif // SIMENTRIES_H
