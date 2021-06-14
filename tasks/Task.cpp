/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include <sstream>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/filesystem.hpp>

/** Frame helper **/
#include <frame_helper/FrameHelper.h>

/** Base types **/
#include <base/samples/IMUSensors.hpp>
#include <base/samples/EventArray.hpp>

#include "Task.hpp"

using namespace davis2pocolog;
namespace fs = boost::filesystem;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    /** Set the img member **/
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    this->img_msg.reset(img);
    img = nullptr;

    /** Set the depth map **/
    ::base::samples::DistanceImage *depth = new ::base::samples::DistanceImage();
    this->depth_msg.reset(depth);
    depth = nullptr;

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

    /** Read calibration **/
    status = this->readCalibration("calib.txt");
    std::cout<<"K:"<<this->K.at<float>(0,0)<<std::endl;

    /** Read the images file (timestamp and filename) **/
    status &= this->readImagesFile("images.txt");

    /** Read the depth file (timestamp and filename) **/
    status &= this->readDepthFile("depthmaps.txt");
    
    /** Write the images **/
    status &= this->writeImages();

    /** Write the events **/
    status &= this->processEvents("events.txt", _events_array_size.value());

    /** Write the imu **/
    status &= this->processIMU("imu.txt");


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

bool Task::readCalibration(const std::string &f1)
{
    fs::path calib_fname = fs::path(this->root_folder)/ fs::path(f1);
    std::ifstream file(calib_fname.string());
    std::string str;
    if (file.is_open())
    { 
        std::getline(file, str);
        std::istringstream iss(str);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>(),
                   std:: back_inserter(tokens));
        std::cout<<"line: "<<str<<std::endl;
        for (auto it=tokens.begin(); it!=tokens.end(); ++it)
            std::cout<<*it<<std::endl;
        this->K = cv::Mat_<float>::eye(3, 3);
        this->K.at<float>(0,0) = std::stof(tokens[0]);
        this->K.at<float>(1,1) = std::stof(tokens[1]);
        this->K.at<float>(0,2) = std::stof(tokens[2]);
        this->K.at<float>(1,2) = std::stof(tokens[3]);
        file.close();
        return true;
    }
    return false;

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
        
bool Task::processEvents(const std::string &filename, const int array_size)
{
    fs::path events_fname = fs::path(this->root_folder)/ fs::path(filename);

    int i = 0;
    std::string str; 
    std::ifstream file(events_fname.string());
    if (!file.is_open())
        return false;
    
    ::base::samples::EventArray events_msg;
    while (std::getline(file, str))
    {
        /** Split the line **/
        std::istringstream iss(str);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>(),
                   std:: back_inserter(tokens));

        ::base::samples::Event ev(
            std::stoi(tokens[1]), std::stoi(tokens[2]),
            ::base::Time::fromSeconds(std::stod(tokens[0])),
            (uint8_t)std::stoi(tokens[3]));

        if (events_msg.events.size() == 0)
        {
            events_msg.time = ev.ts;
        }
 
        std::cout<<"E: "<<ev.ts.toString()<<" "<<ev.x<<" "<<ev.y<<" "<<ev.polarity<<std::endl;
        events_msg.events.push_back(ev);
        if (i%array_size == 0)
        {
            //std::cout<<".";
            events_msg.height = this->img_height;
            events_msg.width = this->img_width;
            this->_events.write(events_msg);
            events_msg.events.clear();
        }
    }

    file.close();

    return true;
}

bool Task::processIMU(const std::string &filename)
{
    fs::path imu_fname = fs::path(this->root_folder)/ fs::path(filename);
    std::ifstream file(imu_fname.string());
    if (!file.is_open())
        return false;
 
    std::string str; 
    std::cout<<"Writing IMU... ";
    while (std::getline(file, str))
    {
        /** Split the line **/
        std::istringstream iss(str);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>(),
                   std:: back_inserter(tokens));
        /** Measurement **/
        ::base::samples::IMUSensors imusamples;
        imusamples.time = ::base::Time::fromMicroseconds(std::stod(tokens[0]));
        imusamples.acc << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]); //[m/s^2]
        imusamples.gyro << std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]); //[rad/s]
        this->_imu.write(imusamples);
    }
    file.close();
    std::cout<<"[DONE]"<<std::endl;
    return true;
}

bool Task::writeImages()
{
    auto it_img =this->image_fname.begin();
    auto it_ts =this->image_ts.begin();

    /** Read first image to know the size (for the array of events) **/
    fs::path img_name_0 = fs::path(this->root_folder)/ fs::path(*it_img);
    cv::Mat img_0 = cv::imread(img_name_0.string(), cv::IMREAD_COLOR);
    cv::Size s = img_0.size();
    this->img_height = s.height;
    this->img_width = s.width;

    /** Write the images **/
    std::cout<<"Writing images... ";
    while(it_img != this->image_fname.end() && it_ts != this->image_ts.end())
    {
        /** The path to he image **/
        fs::path img_name = fs::path(this->root_folder)/ fs::path(*it_img);

        /** Read the image file **/
        cv::Mat img = cv::imread(img_name.string(), cv::IMREAD_COLOR);

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *img_msg_ptr = this->img_msg.write_access();
        img_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(img, *img_msg_ptr);

        /** Write into the port **/
        img_msg_ptr->time = ::base::Time::fromMicroseconds(*it_ts);
        img_msg_ptr->received_time = img_msg_ptr->time;
        this->img_msg.reset(img_msg_ptr);
        _frame.write(this->img_msg);

        ++it_img;
        ++it_ts;
    }
    std::cout<<"[DONE]"<<std::endl;
    return true;
}