/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include <boost/filesystem.hpp>

#include "Task.hpp"

using namespace davis2pocolog;
namespace fs = boost::filesystem;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    this->root_folder = _dataset_path.value();

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    bool status = true;

    status = this->readImagesFile();

    return status;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

bool Task::readImagesFile()
{
    /** Read images timestamps **/
    fs::path img_ts_fname = fs::path(this->root_folder)/ fs::path("images.txt");
    std::ifstream infile;
    infile.open(img_ts_fname.string());
    if (!infile)
    {
        std::cout << "Unable to open file:"<<img_ts_fname.string()<<std::endl;
        return false; // terminate with error
    }

    double ts;
    std::string fname;
    while (infile >> ts)
    {
        this->image_ts.push_back(ts);
        this->image_fname.push_back(fname);
    }
    infile.close();

    return true;
}