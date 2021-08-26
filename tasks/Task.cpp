/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include <sstream>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/filesystem.hpp>

/** OpenEXR **/
#include <ImfInputFile.h>
#include <ImfChannelList.h>
#include <ImfFrameBuffer.h>
#include <ImfRgbaFile.h>

/** Frame helper **/
#include <frame_helper/FrameHelper.h>

/** Base types **/
#include <base/samples/IMUSensors.hpp>
#include <base/samples/EventArray.hpp>

#include "Task.hpp"

using namespace Imf;
using namespace Imath;
using namespace davis2pocolog;
namespace fs = boost::filesystem;

bool ReadEXR(const char *name, float *&rgba, int &xRes, int &yRes, bool &hasAlpha)
{
    /** Example from here:
     * https://gist.github.com/bgotink/635bca8e2a3d47bf6a5f
     * **/
    try {
    InputFile file(name);
    Box2i dw = file.header().dataWindow();
    xRes = dw.max.x - dw.min.x + 1;
    yRes = dw.max.y - dw.min.y + 1;

    half *hrgba = new half[1 * xRes * yRes];

    // for now...
    hasAlpha = true;
    int nChannels = 1;

    hrgba -= 1 * (dw.min.x + dw.min.y * xRes);
    FrameBuffer frameBuffer;
    frameBuffer.insert("Z", Slice(HALF, (char *)hrgba,
				  1*sizeof(half), xRes * 1 * sizeof(half), 1, 1, 0.0));
    file.setFrameBuffer(frameBuffer);
    file.readPixels(dw.min.y, dw.max.y);

    hrgba += 1 * (dw.min.x + dw.min.y * xRes);
    rgba = new float[nChannels * xRes * yRes];
    for (int i = 0; i < nChannels * xRes * yRes; ++i)
	rgba[i] = hrgba[i];
    delete[] hrgba;
    } catch (const std::exception &e) {
        fprintf(stderr, "Unable to read image file \"%s\": %s", name, e.what());
        return NULL;
    }

    return rgba;
}

Task::Task(std::string const& name)
    : TaskBase(name)
{
    /** Set the img member **/
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    this->img_msg.reset(img);
    img = nullptr;

    /** Set the depth_img member **/
    ::base::samples::frame::Frame *depth_img = new ::base::samples::frame::Frame();
    this->depth_img_msg.reset(depth_img);
    depth_img = nullptr;

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
    std::cout<<"K:\n"<<this->K<<std::endl;    

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

    /** Write the twist **/
    status &= this->processTwist("twist.txt");

    /** Write the imu **/
    status &= this->processGroundTruth("groundtruth.txt");

    /** Write the depthmaps **/
    status &= this->writeDepthmaps();

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
        this->K = cv::Mat_<double>::eye(3, 3);
        this->K.at<double>(0,0) = std::stod(tokens[0]);
        this->K.at<double>(1,1) = std::stod(tokens[1]);
        this->K.at<double>(0,2) = std::stod(tokens[2]);
        this->K.at<double>(1,2) = std::stod(tokens[3]);
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
        return true; // terminate with error
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

    std::ifstream file(events_fname.string());
    if (!file.is_open())
        return true;
    
    int i = 0;
    std::string str; 
    ::base::samples::EventArray events_msg;
    std::cout<<"Writing Events...";
    while (std::getline(file, str))
    {
        /** Split the line **/
        std::istringstream iss(str);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>(),
                   std:: back_inserter(tokens));

        ::base::samples::Event ev(
            (uint16_t)std::stoi(tokens[1]), (uint16_t)std::stoi(tokens[2]),
            ::base::Time::fromSeconds(std::stod(tokens[0])),
            (uint8_t)std::stoi(tokens[3]));

        if (events_msg.events.size() == 0)
        {
            events_msg.time = ev.ts;
        }
 
        //std::cout<<"E: "<<ev.ts.toString()<<" "<<ev.x<<" "<<ev.y<<" "<<ev.polarity<<std::endl;
        events_msg.events.push_back(ev);
        if (i%array_size == 0)
        {
            //std::cout<<"events size: "<<events_msg.events.size()<<std::endl;
            events_msg.height = this->img_height;
            events_msg.width = this->img_width;
            RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
            while (status != RTT::WriteStatus::WriteSuccess)
                status = this->_events.write(events_msg);
            events_msg.events.clear();
        }
        ++i;
    }

    /** Write the last event array **/
    if (events_msg.events.size() > 0)
    {
        std::cout<<"last events size: "<<events_msg.events.size();
        events_msg.height = this->img_height;
        events_msg.width = this->img_width;
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            status = this->_events.write(events_msg);
    }
    
    std::cout<<"[DONE]"<<std::endl;

    file.close();

    return true;
}

bool Task::processIMU(const std::string &filename)
{
    fs::path imu_fname = fs::path(this->root_folder)/ fs::path(filename);
    std::ifstream file(imu_fname.string());
    if (!file.is_open())
        return true;//some datasets do not have imu data
 
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
        imusamples.time = ::base::Time::fromSeconds(std::stod(tokens[0]));
        imusamples.acc << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]); //[m/s^2]
        imusamples.gyro << std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]); //[rad/s]
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            status = this->_imu.write(imusamples);
    }
    file.close();
    std::cout<<"[DONE]"<<std::endl;
    return true;
}

bool Task::processTwist(const std::string &filename)
{
    fs::path twist_fname = fs::path(this->root_folder)/ fs::path(filename);
    std::ifstream file(twist_fname.string());
    if (!file.is_open())
        return true;//some datasets do not have twist data
 
    std::string str; 
    std::cout<<"Writing Twist... ";
    while (std::getline(file, str))
    {
        /** Split the line **/
        std::istringstream iss(str);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>(),
                   std:: back_inserter(tokens));
        /** Measurement **/
        ::base::samples::Twist twistsamples;
        twistsamples.time = ::base::Time::fromSeconds(std::stod(tokens[0]));
        twistsamples.frame_id = "camera"; // twist samplas are in camera frame
        twistsamples.linear << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]); //[m/s] velocity
        twistsamples.angular << std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]); //[rad/s]
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            status = this->_twist.write(twistsamples);
    }
    file.close();
    std::cout<<"[DONE]"<<std::endl;
    return true;
}
bool Task::processGroundTruth(const std::string &filename)
{
    fs::path poses_fname = fs::path(this->root_folder)/ fs::path(filename);
    std::ifstream file(poses_fname.string());
    if (!file.is_open())
        return true;//some datasets do not have ground truth pose data
 
    std::string str; 
    std::cout<<"Writing Poses... ";
    while (std::getline(file, str))
    {
        /** Split the line **/
        std::istringstream iss(str);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>(),
                   std:: back_inserter(tokens));
        /** Measurement **/
        ::base::samples::RigidBodyState rbs;
        rbs.time = ::base::Time::fromSeconds(std::stod(tokens[0]));
        rbs.sourceFrame = "cam"; rbs.targetFrame = "world";
        rbs.position << std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]); //[m/s^2]
        rbs.orientation = Eigen::Quaterniond(std::stod(tokens[7]), std::stod(tokens[4]),  std::stod(tokens[5]),  std::stod(tokens[6])); // Eigen expect w, x, y, z
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            status = this->_poses.write(rbs);

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
        img_msg_ptr->time = ::base::Time::fromSeconds(*it_ts);
        img_msg_ptr->received_time = img_msg_ptr->time;
        this->img_msg.reset(img_msg_ptr);
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            status = _frame.write(this->img_msg);

        ++it_img;
        ++it_ts;
    }
    std::cout<<"[DONE]"<<std::endl;
    return true;
}

bool Task::writeDepthmaps()
{
    auto it_depth =this->depth_fname.begin();
    auto it_ts =this->depth_ts.begin();

    // load EXR image
    std::cout<<"Writing depth... ";
    while(it_depth != this->depth_fname.end() && it_ts != this->depth_ts.end())
    {
        fs::path depthmap_name = fs::path(this->root_folder)/ fs::path(*it_depth);
        //std::cout<<depthmap_name.string()<<std::endl;

        float *image;
        int resolution[2];
        bool has_alpha;
        if (!ReadEXR(depthmap_name.string().c_str(), image, resolution[0], resolution[1], has_alpha))
        {
	        printf("couldn't read image %s\n", depthmap_name.string().c_str());
            return false;
        }
        else
        {
            ::base::samples::DistanceImage *depth_msg_ptr = this->depth_msg.write_access();
            depth_msg_ptr->data.clear();
            depth_msg_ptr->height = resolution[1];
            depth_msg_ptr->width = resolution[0];
            depth_msg_ptr->setIntrinsic(this->K.at<double>(0,0),
                            this->K.at<double>(1,1),
                            this->K.at<double>(0,2),
                            this->K.at<double>(1,2));


            for (int i=0; i<resolution[0]*resolution[1]; ++i)
            {
                //std::cout<<"depth[i]"<< image[i] <<std::endl;
                depth_msg_ptr->data.push_back(image[i]);
            }
            /** Write into the port **/
            depth_msg_ptr->time = ::base::Time::fromSeconds(*it_ts);
            this->depth_msg.reset(depth_msg_ptr);
            RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
            while (status != RTT::WriteStatus::WriteSuccess)
                status = _depthmap.write(this->depth_msg);

            free(image);
        }
        ++it_depth;
        ++it_ts;
    }
    std::cout<<"[DONE]"<<std::endl;
    return true;
}

bool Task::writePredictedDepthmaps()
{
    auto it_depth =this->depth_fname.begin();
    auto it_ts =this->depth_ts.begin();

    // load PFM image
    /** Write the images **/
    std::cout<<"Writing depth... ";
    while(it_depth != this->depth_fname.end() && it_ts != this->depth_ts.end())
    {
        /** The path to the depthmap file **/
        fs::path depthmap_name = fs::path(this->root_folder)/ fs::path(*it_depth);

        /** Read the image file **/
        cv::Mat depth = cv::imread(depthmap_name.string(), cv::IMREAD_UNCHANGED);

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *depth_img_msg_ptr = this->depth_img_msg.write_access();
        depth_img_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(depth, *depth_img_msg_ptr);

        /** Write into the port **/
        depth_img_msg_ptr->time = ::base::Time::fromSeconds(*it_ts);
        depth_img_msg_ptr->received_time = depth_img_msg_ptr->time;
        this->depth_img_msg.reset(depth_img_msg_ptr);
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            //status = _depthmap.write(this->depth_img_msg);

        ++it_depth;
        ++it_ts;
    }
    std::cout<<"[DONE]"<<std::endl;
    return true;
}