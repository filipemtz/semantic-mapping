
#ifndef __SEGMAP_SENSORS_H__
#define __SEGMAP_SENSORS_H__

#include <string>
#include <opencv2/opencv.hpp>
#include "libsegmap/types/synchronized_data_package.h"


class SemanticSegmentationLoader
{
public:
    SemanticSegmentationLoader(std::string log_path,
                               std::string data_path="/dados/data");
    ~SemanticSegmentationLoader();

    cv::Mat load(DataSample *sample);

protected:

    std::string _log_data_dir;
};


#endif
