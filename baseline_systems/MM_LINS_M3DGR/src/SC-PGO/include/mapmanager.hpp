#pragma once
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "scancontext/Scancontext.h"

using namespace pcl;
using KeyframepointType = PointCloud<PointType>::Ptr;
using MapType = std::vector<KeyframepointType>;
using keyframePosesType = std::vector<Pose6D>;

struct MapmanagerType
{
    MapType keyframeClouds;                 
    keyframePosesType keyframeposes;        
    keyframePosesType keyframeposesUpdated; 
    SCManager scmanager;                    
    std::vector<double> keyframeTimes;      
    double optimal_matchvalue;

    MapmanagerType() {}
    MapmanagerType(const MapmanagerType &other)
    {
        for (const auto &cloud : other.keyframeClouds)
        {
            boost::shared_ptr<pcl::PointCloud<PointType>> newCloud(new pcl::PointCloud<PointType>(*cloud));
            keyframeClouds.push_back(newCloud);
        }
        keyframeposes = other.keyframeposes;
        keyframeposesUpdated = other.keyframeposesUpdated;
        scmanager = other.scmanager;
        keyframeTimes = other.keyframeTimes;
        optimal_matchvalue = other.optimal_matchvalue;
    }

    MapmanagerType(MapmanagerType &&other) noexcept
    {
        keyframeClouds = std::move(other.keyframeClouds);
        keyframeposes = std::move(other.keyframeposes);
        keyframeposesUpdated = std::move(other.keyframeposesUpdated);
        scmanager = std::move(other.scmanager);
        keyframeTimes = std::move(other.keyframeTimes);
        optimal_matchvalue = std::move(other.optimal_matchvalue);
    }
    MapmanagerType &operator=(const MapmanagerType &other)
    {
        if (this == &other)
        {
            return *this;
        }

        keyframeClouds = other.keyframeClouds;
        keyframeposes = other.keyframeposes;
        keyframeposesUpdated = other.keyframeposesUpdated;
        scmanager = other.scmanager;
        keyframeTimes = other.keyframeTimes;
        optimal_matchvalue = other.optimal_matchvalue;

        return *this;
    }
};

class mapmanager
{
public:
    mapmanager(){
        ActiveMap = std::make_shared<MapmanagerType>();
    };

    void addkeyframewithScancontext(KeyframepointType Keyframe, Pose6D Pose_curr, double KeyframeTimes)
    {
        ActiveMap->keyframeClouds.push_back(Keyframe);
        ActiveMap->keyframeposes.push_back(Pose_curr);
        ActiveMap->scmanager.makeAndSaveScancontextAndKeys(*Keyframe);
        ActiveMap->keyframeTimes.push_back(KeyframeTimes);
        ActiveMap->keyframeposesUpdated.push_back(Pose_curr);
    }

    void act_change()
    {
        SleepMapList.push_back(ActiveMap);
    }

    void act_reset()
    {
        ActiveMap = std::make_shared<MapmanagerType>();
    }
    void clear()
    {
        SleepMapList.clear();
        ActiveMap->keyframeClouds.clear();
        ActiveMap->keyframeposes.clear();
        ActiveMap->keyframeTimes.clear();
        ActiveMap->keyframeposesUpdated.clear();
        ActiveMap->scmanager.polarcontexts_.clear();
        ActiveMap->scmanager.polarcontext_invkeys_.clear();
        ActiveMap->scmanager.polarcontext_vkeys_.clear();
        ActiveMap->scmanager.polarcontext_invkeys_mat_.clear();
        ActiveMap->scmanager.polarcontext_tree_.reset();
        ActiveMap->scmanager.polarcontext_invkeys_to_search_.clear();
    }

    void sleep_delete()
    {
        SleepMapList.erase(SleepMapList.begin() + Wakeup_Mapindex);
    }

    int get_gtsamNum()
    {
        int Num = SleepMapList[Wakeup_Mapindex]->keyframeposes.size();
        return Num;
    }

    int get_Num()
    {
        int Num = Wakeup_Mapindex;
        return Num;
    }

    void transformmap(Eigen::Affine3f trans)
    {
#pragma omp parallel for num_threads(8)
        for (int i = 0; i < ActiveMap->keyframeposes.size(); ++i)
        {
            Eigen::Affine3f prekeyframeposestrans = pcl::getTransformation(ActiveMap->keyframeposes[i].x, ActiveMap->keyframeposes[i].y, ActiveMap->keyframeposes[i].z,
                                                                           ActiveMap->keyframeposes[i].roll, ActiveMap->keyframeposes[i].pitch, ActiveMap->keyframeposes[i].yaw);
            Eigen::Affine3f aftkeyframeposestrans =  trans * prekeyframeposestrans;

            float tx, ty, tz, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(aftkeyframeposestrans, tx, ty, tz, roll, pitch, yaw);
            ActiveMap->keyframeposes[i].x = tx;
            ActiveMap->keyframeposes[i].y = ty;
            ActiveMap->keyframeposes[i].z = tz;
            ActiveMap->keyframeposes[i].roll = roll;
            ActiveMap->keyframeposes[i].pitch = pitch;
            ActiveMap->keyframeposes[i].yaw = yaw;

            Eigen::Affine3f prekeyframeposesUpdatedtrans = pcl::getTransformation(ActiveMap->keyframeposesUpdated[i].x, ActiveMap->keyframeposesUpdated[i].y, ActiveMap->keyframeposesUpdated[i].z,
                                                                                  ActiveMap->keyframeposesUpdated[i].roll, ActiveMap->keyframeposesUpdated[i].pitch, ActiveMap->keyframeposesUpdated[i].yaw);
            Eigen::Affine3f aftkeyframeposesUpdatedtrans = trans * prekeyframeposesUpdatedtrans;

            float tux, tuy, tuz, uroll, upitch, uyaw;
            pcl::getTranslationAndEulerAngles(aftkeyframeposesUpdatedtrans, tux, tuy, tuz, uroll, upitch, uyaw);
            ActiveMap->keyframeposesUpdated[i].x = tux;
            ActiveMap->keyframeposesUpdated[i].y = tuy;
            ActiveMap->keyframeposesUpdated[i].z = tuz;
            ActiveMap->keyframeposesUpdated[i].roll = uroll;
            ActiveMap->keyframeposesUpdated[i].pitch = upitch;
            ActiveMap->keyframeposesUpdated[i].yaw = uyaw;
        }
    }

    void mergemap()
    {
        ActiveMap->keyframeClouds.insert(ActiveMap->keyframeClouds.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeClouds.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeClouds.end());

        ActiveMap->keyframeposes.insert(ActiveMap->keyframeposes.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeposes.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeposes.end());

        ActiveMap->keyframeTimes.insert(ActiveMap->keyframeTimes.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeTimes.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeTimes.end());

        ActiveMap->keyframeposesUpdated.insert(ActiveMap->keyframeposesUpdated.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeposesUpdated.begin(),
                                        SleepMapList[Wakeup_Mapindex]->keyframeposesUpdated.end());

        ActiveMap->scmanager.polarcontexts_.insert(ActiveMap->scmanager.polarcontexts_.begin(),
                                        SleepMapList[Wakeup_Mapindex]->scmanager.polarcontexts_.begin(),
                                            SleepMapList[Wakeup_Mapindex]->scmanager.polarcontexts_.end());

        ActiveMap->scmanager.polarcontext_invkeys_.insert(ActiveMap->scmanager.polarcontext_invkeys_.begin(),
                                        SleepMapList[Wakeup_Mapindex]->scmanager.polarcontext_invkeys_.begin(),
                                        SleepMapList[Wakeup_Mapindex]->scmanager.polarcontext_invkeys_.end());

        ActiveMap->scmanager.polarcontext_vkeys_.insert(ActiveMap->scmanager.polarcontext_vkeys_.begin(),
                                        SleepMapList[Wakeup_Mapindex]->scmanager.polarcontext_vkeys_.begin(),
                                        SleepMapList[Wakeup_Mapindex]->scmanager.polarcontext_vkeys_.end());

        ActiveMap->scmanager.polarcontext_invkeys_mat_.insert(ActiveMap->scmanager.polarcontext_invkeys_mat_.begin(),
                                        SleepMapList[Wakeup_Mapindex]->scmanager.polarcontext_invkeys_mat_.begin(),
                                        SleepMapList[Wakeup_Mapindex]->scmanager.polarcontext_invkeys_mat_.end());

        // rebuild kdtree
        ActiveMap->scmanager.polarcontext_invkeys_to_search_.clear();
        ActiveMap->scmanager.polarcontext_invkeys_to_search_.assign(ActiveMap->scmanager.polarcontext_invkeys_mat_.begin(), ActiveMap->scmanager.polarcontext_invkeys_mat_.end());
        ActiveMap->scmanager.polarcontext_tree_.reset();
        ActiveMap->scmanager.polarcontext_tree_ = std::make_unique<InvKeyTree>(20, ActiveMap->scmanager.polarcontext_invkeys_to_search_, 10);
    }

public:
    std::vector<std::shared_ptr<MapmanagerType>> SleepMapList; // sleepmap
    std::shared_ptr<MapmanagerType> ActiveMap;                 // activemap
    int Wakeup_Mapindex;
};
