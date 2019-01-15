#include "solid_sync_collider.h"


SolidSyncCollider::SolidSyncCollider(unsigned int threadCnt)
{
    _threadCnt = threadCnt;
    assert(threadCnt <= MAX_THREAD_CNT);

    for (unsigned int i = 0; i < _threadCnt; i++) {
        colliders.push_back(std::make_shared<SolidCollider>());
        colliderMutexes[i].unlock();
    }

}

void SolidSyncCollider::init(std::vector<std::vector<std::string>> groupedModelPaths)
{
    for (unsigned int i = 0; i < _threadCnt; i++) {
        colliders.at(i)->init(groupedModelPaths);
        colliderMutexes[i].unlock();
    }
}

void SolidSyncCollider::paint(std::vector<Eigen::Matrix4d> matrices,bool onlyRobot)
{
    (*colliders.begin())->paint(matrices,onlyRobot);
}


bool SolidSyncCollider::isCollided(std::vector<Eigen::Matrix4d> matrices)
{
    while (true) {
        for (unsigned i = 0; i < _threadCnt; i++) {
            //bool opened = colliderMutexes->try_lock();
//            if (opened)colliderMutexes->unlock();
//            info_msg("mutex ", i, opened ? " opened" : " closed");

            if (colliderMutexes[i].try_lock()) {
                bool result = colliders.at(i)->isCollided(matrices);
                colliderMutexes[i].unlock();
                return result;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

}

bool SolidSyncCollider::isCollided(std::vector<Eigen::Matrix4d> matrices, int robotNum)
{
    while (true) {
        for (unsigned i = 0; i < _threadCnt; i++) {
            //bool opened = colliderMutexes->try_lock();
//            if (opened)colliderMutexes->unlock();
//            info_msg("mutex ", i, opened ? " opened" : " closed");

            if (colliderMutexes[i].try_lock()) {
                bool result = colliders.at(i)->isCollided(matrices,robotNum);
                colliderMutexes[i].unlock();
                return result;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}
std::vector<float> SolidSyncCollider::getPoints(std::vector<Eigen::Matrix4d> matrices)
{
    colliders.front()->getPoints(matrices);

}
std::vector<double> SolidSyncCollider::getBoxChords(unsigned long objNum, std::vector<Eigen::Matrix4d> matrices)
{
    return colliders.front()->getBoxChords(objNum, matrices);
}
float SolidSyncCollider::getDistance(std::vector<Eigen::Matrix4d> matrices)
{
    return colliders.front()->getDistance(matrices);
}

