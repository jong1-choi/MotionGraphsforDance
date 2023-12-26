#ifndef motion_hpp
#define motion_hpp

#include <stdio.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include <string.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "GLTools.hpp"

using namespace glm;
using namespace std;

struct Link {
    glm::vec3 parentGlobalP = glm::vec3(0);
    glm::quat parentGlobalQ = glm::quat(1, 0, 0, 0);
    glm::vec3 l = glm::vec3(0);
    glm::quat q = glm::quat(1, 0, 0, 0);
    glm::vec3 v = glm::vec3(0);
    float theta = 0;
    float yRot = 0;
    
//    bool isBeat = false;
    bool isHand = false;
    bool isEnd = false;
    int parentIndex = -1;
    int childIndex = -1;

    void render() const {
        drawSphere( getPos(), 0.2);
//        if(isBeat){
//            drawSphere( getPos(), 0.2, glm::vec4(1,0,1,0) );
////            drawSphere( parentGlobalP, 0.2, glm::vec4(1,0,1,0) );
//        }
//        else {
//            drawSphere( getPos(), 0.2);
////            drawSphere( parentGlobalP, 0.2);
//        }
        
        drawCylinder( getPos(), parentGlobalP, 0.1 );
    }
    void shapeRender(glm::vec3 offset, glm::vec4 color = glm::vec4(1,0,0,0), float size = 0.1, float multiple = 2) const {
        drawCylinder( multiple*(getPos() + offset), multiple*(parentGlobalP + offset), size, color);
    }
    glm::vec3 getPos() const {
        return parentGlobalQ * l + parentGlobalP;
    }
    glm::vec3 getPos(float multiplier) const {
        if(parentIndex == -1) return l * q;
        return parentGlobalQ * l * multiplier + parentGlobalP;
    }
    glm::vec3 getNearP(glm::vec3 dir, float scale = 1) const {
        return getOri() * dir * scale * 0.2f + getPos();
    }
    glm::quat getOri() const {
        return parentGlobalQ * q;
    }
    void rotate( const glm::quat& rot ) {
        q = rot * q;
    }
    void updatePose( const glm::vec3& pos, const glm::quat& ori ) {
        parentGlobalP = pos;
        parentGlobalQ = ori;
    }
    Link( int parI, int chiN, const glm::vec3& ll, const glm::quat& qq, const float yR, const glm::vec3& pl, const glm::quat& pq, bool end, bool h) : parentIndex(parI), childIndex(chiN), l(ll), q(qq), yRot(yR), parentGlobalP(pl), parentGlobalQ(pq), isEnd(end), isHand(h){}
    Link( int parI, int chiN, const glm::vec3& ll, const glm::quat& qq) : parentIndex(parI), childIndex(chiN), l(ll), q(qq){}
    
};


struct Body {
    std::vector<Link> links;
    std::vector<pair<float,glm::vec3>> points;
    std::vector<pair<float,glm::vec3>> pc;
    glm::vec3 globalP = glm::vec3(0);
    glm::quat globalQ = glm::quat(1, 0, 0, 0);
//    Eigen::MatrixXf displacement;
    bool isReadyPose = false;
    bool constraint = false;
    bool isBeat = false;
    bool isMeter = false;
    bool isStart = false;
    bool isEnd = false;
    bool isTransition = false;
    
    int test = 0;
    int endMeter;
    
    ~Body() {
        clear();
    }
    void clear() {
        links.clear();
        vector<Link>().swap(links);
        points.clear();
        vector<pair<float,glm::vec3>>().swap(points);
        pc.clear();
        vector<pair<float,glm::vec3>>().swap(pc);
        globalP = glm::vec3(0);
        globalQ = glm::quat(1, 0, 0, 0);
    }
    void add( int parI, int chiI, const glm::vec3& ll, const glm::quat& qq, const float yR, const glm::vec3& pl, const glm::quat& pq, bool end, bool h ) {
            links.push_back(Link(parI, chiI, ll, qq, yR, pl, pq, end, h));
    }
    void add( int parI, int chiI, const glm::vec3& ll, const glm::quat& qq) {
            links.push_back(Link(parI, chiI, ll, qq));
    }
    void render() const {
        for (auto& l : links) {
            if(l.parentIndex >= 0)
                l.render();
        }
    }
    
    void shapeRender(glm::vec3 offset = glm::vec3(0), glm::vec4 color = glm::vec4(1,0,0,0), float size = 0.2) const {
        int i = -1;
        for (auto& l : links) {
            i++;
//            if(i >= 10 && i <= 29) {
//                continue;
//            }
//            if(i >= 34 && i <= 53) {
//                continue;
//            }
            
            if(l.parentIndex >= 0)
                l.shapeRender(offset, color, size);
        }
    }
    
    
    void pcRender() const {
        for(auto &p : pc) {
            drawSphere( p.second, 0.1 );
        }
    }
    
    const std::vector<int> getAncestors( int end ) {
        std::vector<int> ret;
        int index = end;
        bool endCheck = 1;
        
        while(endCheck){
            if(links[index].parentIndex == -1)
                endCheck = false;
            else{
                ret.push_back(links[index].parentIndex);
                index = links[index].parentIndex;
            }
        }
        return ret;
    }
    void updateLink(){
        for (int i = 1; i < links.size(); i++) {
            Link pLink = links[links[i].parentIndex];
            links[i].updatePose(pLink.getPos(), pLink.getOri());
        }
    }
    void addPC(Body &get, const Body &give, float w) {
        for(auto &p : give.points) {
            get.pc.push_back({w*p.first , p.second});
        }
    }
};

struct Motion {
    std::vector<Body> bodies;
    unsigned int motionNum;
    unsigned int frameNum;
    
    void clear() {
        bodies.clear();
        vector<Body>().swap(bodies);
    }
    void add(const Body &body) {
        bodies.push_back(body);
    }
    

};

#endif /* motion_hpp */
