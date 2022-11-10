#include "simentries.h"
#include "robotvacuumsim.h"

struct ExampleEntry
{
    int m_menuLevel;
    const char* m_name;
    const char* m_description;
    CommonExampleInterface::CreateFunc* m_createFunc;
    int m_option;

    ExampleEntry(int menuLevel, const char* name)
        : m_menuLevel(menuLevel), m_name(name), m_description(0), m_createFunc(0), m_option(0)
    {
    }

    ExampleEntry(int menuLevel, const char* name, const char* description, CommonExampleInterface::CreateFunc* createFunc, int option = 0)
        : m_menuLevel(menuLevel), m_name(name), m_description(description), m_createFunc(createFunc), m_option(option)
    {
    }
};

static ExampleEntry simExamples[] =
{
    ExampleEntry(0, "Mapping"),

    ExampleEntry(1, "2D Occupancy Grid", "Drive around with the arrow keys. As the onboard LiDar measures surfaces, the robot generates a probabilistic map", RobotVacuumSimCreateFunc)
};

struct ExampleEntriesInternalData
{
    btAlignedObjectArray<ExampleEntry> m_allExamples;
};

SimEntries::SimEntries()
{
    m_data = new ExampleEntriesInternalData;
}

SimEntries::~SimEntries()
{
    delete m_data;
}

void SimEntries::initExampleEntries()
{
    m_data->m_allExamples.clear();



    int numDefaultEntries = sizeof(simExamples) / sizeof(ExampleEntry);
    for (int i = 0; i < numDefaultEntries; i++)
    {
        m_data->m_allExamples.push_back(simExamples[i]);
    }

}

void SimEntries::initOpenCLExampleEntries()
{

}

int SimEntries::getNumRegisteredExamples()
{
    return m_data->m_allExamples.size();
}

CommonExampleInterface::CreateFunc* SimEntries::getExampleCreateFunc(int index)
{
    return m_data->m_allExamples[index].m_createFunc;
}

int SimEntries::getExampleOption(int index)
{
    return m_data->m_allExamples[index].m_option;
}

const char* SimEntries::getExampleName(int index)
{
    return m_data->m_allExamples[index].m_name;
}

const char* SimEntries::getExampleDescription(int index)
{
    return m_data->m_allExamples[index].m_description;
}
