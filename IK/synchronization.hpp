//
//  synchronization.hpp
//  IK
//
//  Created by 최종원 on 2023/11/25.
//

#ifndef synchronization_hpp
#define synchronization_hpp

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio/miniaudio.h"
#include "AudioFile/AudioFile.h"
#include <stdio.h>

#define WINDOW_SIZE 9

int getMusicLength(){
    AudioFile<double> audioFile;
    audioFile.load("/Users/choijongwon/Downloads/featureData/Deep Silencio - Underwater.wav");
    return int(audioFile.getLengthInSeconds());
}

Motion makeTransition(const Motion &beforeM, const Motion &afterM, int w) {
    Motion transition;
    
    for(float i = 1; i < w; i++) {
        float a = 2*(i/w)*(i/w)*(i/w) - 3*(i/w)*(i/w) + 1;
        int j = 0;
        Body b;
        
        for(auto &l : afterM.bodies[i-1].links){
            b.add(l.parentIndex, l.childIndex, a * (beforeM.bodies[beforeM.frameNum-1].links[j].l) + (1-a) * (l.l), glm::slerp(beforeM.bodies[beforeM.frameNum-1].links[j].q, l.q, 1-a));
            j++;
        }
        b.updateLink();
        b.isTransition = true;
        transition.add(b);
    }
    transition.frameNum = transition.bodies.size();
    return transition;
}

Motion motionWarping(Motion & m, int f){
    Motion motion;
    for(int i = 0; i < f; i++){
        Body b;
        int j = 0;
        float interval = i * (m.bodies.size()-1) / float(f);
        float a = interval - int(interval);
        a = 2*a*a*a - 3*a*a + 1;
        
        for(auto &l : m.bodies[int(interval)].links){
            b.add(l.parentIndex, l.childIndex, a * (l.l) + (1-a) * (m.bodies[int(interval)+1].links[j].l), glm::slerp(l.q, m.bodies[int(interval)+1].links[j].q, 1-a));
            j++;
        }
        b.updateLink();
        
        if(m.bodies[int(interval)].isBeat || m.bodies[int(interval)].isBeat) b.isBeat = true;
        if(i == f-1) b.isBeat = true;
        motion.add(b);
    }
    motion.frameNum = motion.bodies.size();
    return motion;
}

Motion setMotion(const vector<Motion> &motions, const Node * n, const int targetF, int state){
    Motion m;
    int extraF = (n->endFrame - n->startFrame)/3;
//    std::cout << n->endFrame - n->startFrame << " " << extraF << std::endl;
    if(state == 0)
        for(int i = n->startFrame; i < n->endFrame ; i++) {
            m.add(motions[n->motionNum].bodies[i]);
        }
    else if(state == 1)
        for(int i = n->startFrame - extraF; i < n->endFrame; i++) {
            m.add(motions[n->motionNum].bodies[i]);
        }
    else if(state == 2)
        for(int i = n->startFrame - extraF; i < n->endFrame + extraF; i++) {
            m.add(motions[n->motionNum].bodies[i]);
        }
    else if(state == 3)
        for(int i = n->startFrame; i < n->endFrame + extraF; i++) {
            m.add(motions[n->motionNum].bodies[i]);
        }
    m.frameNum = m.bodies.size();
    m = motionWarping(m, targetF);
    return m;
}

void motionBlending(const Motion &beforeM, Motion &afterM, int w) {
    glm::vec3 bRootOri = normalize(beforeM.bodies[beforeM.frameNum-1].links[0].q * glm::vec3(0,0,1));
    glm::vec3 aRootOri = normalize(afterM.bodies[0].links[0].q * glm::vec3(0,0,1));
    
    glm::vec3 bOriXZ = normalize(glm::vec3(bRootOri.x, 0, bRootOri.z));
    glm::vec3 aOriXZ = normalize(glm::vec3(aRootOri.x, 0, aRootOri.z));

    float angleOff;
    if(cross(aOriXZ, bOriXZ).y > 0) angleOff = acos(dot(bOriXZ, aOriXZ));
    else angleOff = -acos(dot(bOriXZ, aOriXZ));

    glm::vec3 bRootPos = beforeM.bodies[beforeM.frameNum-1].links[0].l;
    glm::vec3 aRootPos = afterM.bodies[0].links[0].l;

    glm::vec3 bXZPos = glm::vec3(bRootPos.x, 0, bRootPos.z);
    glm::vec3 posOff = bRootPos - aRootPos;
    posOff = glm::vec3(posOff.x, 0, posOff.z);

    for(auto &b : afterM.bodies) {
        glm::vec3 offVec = b.links[0].l - aRootPos;
        offVec = glm::vec3(offVec.x, 0, offVec.z);
        b.links[0].l = glm::vec3(0,b.links[0].l.y,0) + bXZPos + glm::angleAxis(angleOff, glm::vec3(0,1,0)) * offVec;
        b.links[0].q = glm::angleAxis(angleOff, glm::vec3(0,1,0)) * b.links[0].q;
        b.updateLink();
    }
}

Motion syncMotions(MotionGraph &g, const vector<Motion> &motions, int bpm, int fps, int w) {
    Motion m;
//    int targetFrame = w/2 + fps * 4 * 60 / float(bpm);
//    int targetFrame = fps * 4 * 60 / float(bpm);
    int targetFrame = 60;
    int duration = getMusicLength();
    Motion beforeM;
    Motion afterM;
    Motion transM;
    Node * after;
    Node * before = g.nodeMap.find(g.findEdge(32,129))->second;
    
    beforeM = setMotion(motions, before, 3*targetFrame/4, 0);
    
    for(int i = 0; i < beforeM.frameNum; i++){
        m.add(beforeM.bodies[i]);
    }
    
    for(int i = 1; i < duration/int(4 * 60 / float(bpm)); i++){
        int nextNum = rand() % before->end->candidates.size();
        after = g.nodeMap.find(before->end->candidates[nextNum].first)->second;
        afterM = setMotion(motions, after, targetFrame, 1);
        motionBlending(beforeM, afterM, w);
        transM = makeTransition(beforeM, afterM, w);
        std::cout << transM.bodies.size() << std:: endl;
        std::cout << afterM.frameNum - w - 2 << std:: endl;
        for(auto &t : transM.bodies) m.add(t);
        for(int j = w-1; j < afterM.frameNum; j++) m.add(afterM.bodies[j]);
        beforeM.clear();
        beforeM = afterM;
        afterM.clear();
        before = after;
    }
    return m;
}

#endif /* synchronization_hpp */
