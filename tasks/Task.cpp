/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include <boost/filesystem.hpp>
#include <sstream>

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

    /** Read the images file (timestamp and filename) **/
    status = this->readImagesFile("images.txt");

    /** Read the depth file (timestamp and filename) **/
    status = this->readDepthFile("depthmaps.txt");

    /** Write the events **/

    /** Write the imu **/

    /** Write the images **/

    /** Write the depthmaps **/


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

bool Task::readImagesFile(const std::string &filename)
{
    /** Read images timestamps **/
    fs::path img_ts_fname = fs::path(this->root_folder)/ fs::path(filename);
    std::ifstream infile;
    infile.open(img_ts_fname.string());
    if (!infile)
    {
        std::cout << "Unable to open file:"<<img_ts_fname.string()<<std::endl;
        return false; // terminate with error
    }

    std::string sentence;
    int i = 0;
    while (infile >> sentence)
    {
        if((i%2) == 0)
            this->image_ts.push_back(std::stod(sentence));
        else
            this->image_fname.push_back(sentence);
        i++;
    }
    infile.close();

    return true;
}

bool Task::readDepthFile(const std::string &filename)
{
    /** Read images timestamps **/
    fs::path img_ts_fname = fs::path(this->root_folder)/ fs::path(filename);
    std::ifstream infile;
    infile.open(img_ts_fname.string());
    if (!infile)
    {
        std::cout << "Unable to open file:"<<img_ts_fname.string()<<std::endl;
        return false; // terminate with error
    }

    std::string sentence;
    int i = 0;
    while (infile >> sentence)
    {
        if((i%2) == 0)
            this->depth_ts.push_back(std::stod(sentence));
        else
            this->depth_fname.push_back(sentence);
        i++;
    }
    infile.close();

    return true;
}