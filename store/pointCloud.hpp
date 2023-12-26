#ifndef pointCloud_hpp
#define pointCloud_hpp

#include <stdio.h>
#include "motion.hpp"
#include "pfm.hpp"
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION

namespace PC{


void copyMotion(const vector<Motion> &origin, vector<Motion> &copy) {
    std::copy(origin.begin(), origin.end(), std::back_inserter(copy));
}

void alignRoot(vector<Motion> &motions) {
    for(auto &motion : motions) {
        for(auto &body : motion.bodies) {
            body.links[0].l = glm::vec3(0, body.links[0].l.y, 0);
            body.links[0].q = inverse(glm::angleAxis(body.links[0].yRot*2, glm::vec3(0,1,0))) * body.links[0].q;
            body.updateLink();
        }
    }
}
void createPoint(vector<Motion> &motions, float w){
    for(auto &motion : motions) {
        for(auto &body : motion.bodies) {
            for(auto &link : body.links) {
                if(link.isHand || link.isEnd) continue;
                body.points.push_back({ w, link.getPos() });
                body.points.push_back({ w, link.getPos(0.5) });
                body.points.push_back({ w, link.getNearP(glm::vec3(1,0,0)) });
                body.points.push_back({ w, link.getNearP(glm::vec3(0,1,0)) });
                body.points.push_back({ w, link.getNearP(glm::vec3(0,0,1)) });
                
            }
        }
    }
}
void createPC(vector<Motion> &motions, int windowSize) {
    for(auto &motion : motions) {
        for(int i = 0; i < motion.frameNum; i++) {
            for(int j = -windowSize/2; j <= windowSize/2; j++){
                if( (i + j) < 0 )
                    motion.bodies[i].addPC( motion.bodies[i], motion.bodies[0], 1.f / (1 + abs(j)) );
                else if( (i + j) >= motion.frameNum )
                    motion.bodies[i].addPC( motion.bodies[i], motion.bodies[motion.frameNum-1], 1.f / (1 + abs(j)) );
                else
                    motion.bodies[i].addPC( motion.bodies[i], motion.bodies[i+j], 1.f / (1 + abs(j)) );
            }
        }
    }
}
void readyPoseCheck(vector<Motion> &copyMotions, vector<Motion> &motions) {
    float dist;
    int frame = 0;
    for(auto &m : copyMotions){
        for(auto &b : m.bodies) {
            dist = 0;
            for(int i = 0; i < b.pc.size(); i++){
                dist += glm::length(b.points[i % b.points.size()].second - b.pc[i].second) / b.pc.size();
                
            }
            if(dist < 0.06) b.isReadyPose = true;
            frame++;
        }
    }
    for(int i = 0; i < copyMotions.size(); i++){
        for(int j = 0; j < copyMotions[i].bodies.size(); j++){
            if(copyMotions[i].bodies[j].isReadyPose) motions[i].bodies[j].isReadyPose = true;
        }
    }
}
void getDistance(const Motion &motion1, const Motion &motion2, Eigen::MatrixXf &distanceMap, int num1, int num2){
    float x1Bar{0},z1Bar{0},x2Bar{0},z2Bar{0};
    float up1{0},up2{0},down1{0},down2{0};
    float x0{0},z0{0};
    float theta{0};
    float weightSum{0}, distSum{0}, distTemp{0};
    int x{-1},y{-1};
    glm::vec4 newP{0};
    glm::mat4 transform(1.0f);
    distanceMap = Eigen::MatrixXf::Zero(motion1.bodies.size(),motion2.bodies.size());
    

    for(auto &pc : motion1.bodies[10].pc){
        weightSum += pc.first;
    }
    for(auto &body1 : motion1.bodies) {
        y++;
        x = -1;
        x1Bar = 0;
        z1Bar = 0;
        for(auto &p : body1.pc) {
            x1Bar += p.first * p.second.x;
            z1Bar += p.first * p.second.z;
        }
        for(auto &body2 : motion2.bodies) {
            x++;
            x2Bar = 0;
            z2Bar = 0;
            up1 = 0;
            up2 = 0;
            down1 = 0;
            down2 = 0;
            distSum = 0;
            transform = glm::mat4(1.0f);
            for(auto &p : body2.pc) {
                x2Bar += p.first * p.second.x;
                z2Bar += p.first * p.second.z;
            }
            for(int i = 0; i < body1.pc.size(); i++) {
//                std::cout << i << std::endl;
                
                up1 += body1.pc[i].first
                    * ( body1.pc[i].second.x * body2.pc[i].second.z
                    - body2.pc[i].second.x * body1.pc[i].second.z );
                down1 += body1.pc[i].first
                    * ( body1.pc[i].second.x * body2.pc[i].second.x
                    + body1.pc[i].second.z * body2.pc[i].second.z );
            }
            up2 = 1/weightSum * ( x1Bar*z2Bar - x2Bar*z1Bar );
            down2 = 1/weightSum * ( x1Bar*x2Bar + z1Bar*z2Bar );
            theta = atan( (up1 - up2)/(down1 - down2) );
            x0 = 1/weightSum * ( x1Bar - x2Bar*glm::cos(theta) - z2Bar*glm::sin(theta) );
            z0 = 1/weightSum * ( z1Bar + x2Bar*glm::sin(theta) - z2Bar*glm::cos(theta) );

            transform = glm::rotate(transform, glm::radians(theta), glm::vec3(0.0f, 1.0f, 0.0f));
            transform = glm::translate(transform, glm::vec3(x0, 0.0f, z0));
            
            for(int i = 0; i < body1.pc.size(); i++) {
                newP = transform * glm::vec4(body2.pc[i].second, 1.0f);
                distTemp = (body1.pc[i].second.x - newP.x)*(body1.pc[i].second.x - newP.x)
                        + (body1.pc[i].second.y - newP.y)*(body1.pc[i].second.y - newP.y)
                        + (body1.pc[i].second.z - newP.z)*(body1.pc[i].second.z - newP.z);
                distSum += distTemp / body1.pc.size();
            }
            
            distanceMap(y,x) = distSum/20.f;
//            if(distSum > 1)
//                distanceMap(y,x) = 1;
//            else
                
            
//            if(body1.isReadyPose || body2.isReadyPose) distanceMap(y,x) = 1;
//            if(!body1.isBeat || !body2.isBeat) distanceMap(y,x) = 1;

        }
    }
    std::string path = "/Users/choijongwon/Downloads/featureData/distanceMap1/";
    path.append(to_string(num1));
    path.append("-");
    path.append(to_string(num2));
    path.append(".pfm");
    
    savePFM(path, distanceMap.rows(), distanceMap.cols(), 1, distanceMap.data());
}

void getLocalMinima(Eigen::MatrixXf &distanceMap, float threshold, int wSize, int num1, int num2){
    Eigen::MatrixXf featureMap = Eigen::MatrixXf::Zero( distanceMap.rows(), distanceMap.cols() );

//    for(int i = 1; i < distanceMap.rows()-1; i++) for(int j = 1; j < distanceMap.cols()-1; j++){
//        bool isMin = true;
//        for(int x = -1; x <= 1; x++) for(int y = -1; y <= 1; y++) {
////            if(x==0 && y==0) continue;
//            if( distanceMap(i+y,j+x) < distanceMap(i,j) ){
//                isMin = false;
//            }
//        }
//        if( isMin && distanceMap(i,j) < threshold ) featureMap(i,j) = 1;
//    }
    for(int i = 1; i < distanceMap.rows()-1; i++) for(int j = 1; j < distanceMap.cols()-1; j++){
        if( distanceMap(i,j) < threshold ) featureMap(i,j) = 1;
    }
    
    for(int i = 0; i < distanceMap.rows(); i++) for(int j = 0; j < distanceMap.cols(); j++) {
        featureMap(i,j) *= (1 - distanceMap(i,j));
//        distanceMap(i,j) += featureMap(i,j);
    }
    
    std::string path = "/Users/choijongwon/Downloads/featureData/featureMap3/";
    path.append(to_string(num1));
    path.append("-");
    path.append(to_string(num2));
    path.append(".pfm");
    
    savePFM(path, featureMap.rows(), featureMap.cols(), 1, featureMap.data());
}
}
#endif /* pointCloud_hpp */
