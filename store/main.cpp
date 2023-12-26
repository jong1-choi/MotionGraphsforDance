#define GL_SILENCE_DEPRECATION
#define STB_IMAGE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <iostream>
#include <JGL/JGL_Window.hpp>
#include <thread>
#include "AnimView.hpp"
#include "GLTools.hpp"
#include "graph.hpp"
#include "pointCloud.hpp"
#include "beatChecker.hpp"
#include "synchronization.hpp"
 
#define PI 3.141592f
#define WINDOW_SIZE_1 7
#define WINDOW_SIZE_2 27
#define TESTNUM 2
 
float frameTime = 0;
unsigned int frameNum = 0;
AnimView* animView;
vector< BVH * > bvh;
vector< Motion > motions;
vector< Motion > copyMotions;
MotionGraph graph;
std::string path = "/Users/choijongwon/Downloads/featureData/cleanup_bvh";
//std::string path = "/Users/choijongwon/Downloads/featureData/30fps";
pair<int,int> s = {3,410};
Node* before;
Node* after;
bool isTransition = false;
Motion trans;
Motion beforeM;
Motion afterM;
glm::vec3 globalPos = glm::vec3(0);
glm::vec3 offset = glm::vec3(0);
float globalRot = 0;
Motion testMotion;

/// 논문용 테스트
Motion testMotion1;
Motion testMotion2;

void getFile(std::string path, vector< BVH * > &bvh) {
    for( const auto &file : std::filesystem::directory_iterator(path)){
        bvh.push_back(new BVH);
        bvh.back()->Load( file.path().string().c_str() );
    }
}

void init(std::string path, vector< BVH * > &bvh, vector<Motion> &motions, int f) {
    getFile(path, bvh);
    for( int i = 0; i < bvh.size(); i++ ) {
        Motion temp;
        int k = 0;
        int fps = int( 1 / bvh[i]->interval );
        motions.push_back(temp);
        for(int j = 0; j < bvh[i]->num_frame; j++) {
            if(fps != f && j % (fps/f) != 0) continue;
            Body body;
            bvh[i]->UpdatePose(j,body,0.1);
            motions[i].add(body);
//            motions[i].bodies[j].updateLink();
            motions[i].bodies[k++].updateLink();
        }
        motions[i].frameNum = motions[i].bodies.size();
        bvh[i]->Clear();
    }
}

//Motion makeTransition(const Motion &beforeM, const Motion &afterM, int w) {
//    Motion transition;
//
//    for(float i = 1; i < w; i++) {
//        float a = 2*(i/w)*(i/w)*(i/w) - 3*(i/w)*(i/w) + 1;
//        int j = 0;
//        Body b;
//
//        for(auto &l : beforeM.bodies[beforeM.frameNum - w + i].links){
//            b.add(l.parentIndex, l.childIndex, a * (l.l) + (1-a) * (afterM.bodies[i-1].links[j].l), glm::slerp(l.q, afterM.bodies[i-1].links[j].q, 1-a));
//            if(l.isHand) b.links[j].isHand = true;
//            if(l.isEnd) b.links[j].isEnd = true;
//            j++;
//        }
//        b.updateLink();
//        b.isTransition = true;
//        transition.add(b);
//    }
//    transition.frameNum = transition.bodies.size();
//    return transition;
//}

void frame(float dt) {
    frameTime += dt;
    if(frameTime > 1/30.f){
        frameNum ++;
        frameTime-=1/30.f;
//        if( frameNum%60==0 ) printf(".");

//        if(!isTransition && frameNum == beforeM.frameNum - 2*WINDOW_SIZE_1){
//            frameNum = 0;
//            if(before->end->candidates.size() == 0)
//                std::cout << "candidates size is 0" << std::endl;
//            int num1 = rand() % before->end->candidates.size();
//            Edge * nextE = before->end->candidates[num1].first;
//
////            int num2 = rand() % graph.nodeMap.find(nextE)->second.size();
//            after = graph.nodeMap.find(nextE)->second;
//
////            std::cout << "before Node -> motion: " << before->motionNum << ", start: " << before->startFrame << ", end: " << before->endFrame << ", frameNum: " << beforeM.frameNum << std::endl;
////            std::cout << "before node's candidates" << std::endl;
////            for(auto &c : before->end->candidates) {
////                std::cout << "(" << c.first->index.first << "," << c.first->index.second << "), isUse : " << c.first->isUse << ", isMeter : " << motions[c.first->index.first].bodies[c.first->index.second].isMeter << std::endl;
////            }
////            std::cout << "next edge is " << num1 << ", motion: " << nextE->index.first << ", frame: " << nextE->index.second << std::endl;
////            for(auto &n : graph.nodeMap.find(nextE)->second) {
////                std::cout << "motion -> " << n->motionNum << " start frame -> " << n->startFrame << " end frame -> " << n->endFrame << std::endl;
////            }
////            std::cout << std::endl;
//
//            for(int i = after->startFrame - WINDOW_SIZE_1; i < after->endFrame + WINDOW_SIZE_1; i++) {
//                afterM.add(motions[after->motionNum].bodies[i]);
//            }
//            afterM.frameNum = after->endFrame - after->startFrame + 2*WINDOW_SIZE_1;
//
////            glm::vec3 bRootOri = normalize(beforeM.bodies[beforeM.frameNum - WINDOW_SIZE_1/2 - 1].links[0].q * glm::vec3(0,0,1));
////            glm::vec3 aRootOri = normalize(afterM.bodies[WINDOW_SIZE_1/2].links[0].q * glm::vec3(0,0,1));
//            glm::vec3 bRootOri = normalize(beforeM.bodies[beforeM.frameNum - WINDOW_SIZE_1 - 1].links[0].q * glm::vec3(0,0,1));
//            glm::vec3 aRootOri = normalize(afterM.bodies[WINDOW_SIZE_1].links[0].q * glm::vec3(0,0,1));
//
//            glm::vec3 bOriXZ = normalize(glm::vec3(bRootOri.x, 0, bRootOri.z));
//            glm::vec3 aOriXZ = normalize(glm::vec3(aRootOri.x, 0, aRootOri.z));
//
//            float angleOff;
//            if(cross(aOriXZ, bOriXZ).y > 0) angleOff = acos(dot(bOriXZ, aOriXZ));
//            else angleOff = -acos(dot(bOriXZ, aOriXZ));
//
////            glm::vec3 bRootPos = beforeM.bodies[beforeM.frameNum - WINDOW_SIZE_1/2 - 1].links[0].l;
////            glm::vec3 aRootPos = afterM.bodies[WINDOW_SIZE_1/2].links[0].l;
//            glm::vec3 bRootPos = beforeM.bodies[beforeM.frameNum - WINDOW_SIZE_1 - 1].links[0].l;
//            glm::vec3 aRootPos = afterM.bodies[WINDOW_SIZE_1].links[0].l;
//
//            glm::vec3 bXZPos = glm::vec3(bRootPos.x, 0, bRootPos.z);
//            glm::vec3 posOff = bRootPos - aRootPos;
//            posOff = glm::vec3(posOff.x, 0, posOff.z);
////            glm::vec3 origin = afterM.bodies[0].links[0].l;
//
//            for(auto &b : afterM.bodies) {
//                glm::vec3 offVec = b.links[0].l - aRootPos;
//                offVec = glm::vec3(offVec.x, 0, offVec.z);
//                b.links[0].l = glm::vec3(0,b.links[0].l.y,0) + bXZPos + glm::angleAxis(angleOff, glm::vec3(0,1,0)) * offVec;
//                b.links[0].q = glm::angleAxis(angleOff, glm::vec3(0,1,0)) * b.links[0].q;
//                b.updateLink();
//            }
//            afterM = motionWarping(afterM, 60);
//
//            trans = makeTransition(beforeM, afterM, WINDOW_SIZE_1);
//            isTransition = true;
//        }
//        else if(isTransition && frameNum == trans.frameNum) {
//            frameNum = 0;
//            beforeM.clear();
//            beforeM = afterM;
//            afterM.clear();
//            before = after;
//            isTransition = false;
//        }
    }
}

void render() {
    
//    motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].shapeRender(glm::vec4(0,0,0,0), glm::vec4(1,0,0,0));
    
    testMotion.bodies[frameNum % testMotion.bodies.size()].shapeRender(glm::vec4(0,0,0,0), glm::vec4(1,0,0,0));
    
//    if(!motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].isBeat)
//        motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].shapeRender();
//    else
//        motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].shapeRender(glm::vec4(0,0,1,0));
    //
//    if(frameNum == 2 || frameNum == 7 || frameNum == 17 || frameNum == 21 || frameNum == 120 || frameNum == 180)
//        motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].shapeRender(glm::vec4(0,0,0,0), glm::vec4(1,0,0,0));
//    else if(frameNum == 52 || frameNum == 55 || frameNum == 60 || frameNum == 80 || frameNum == 96)
//        motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].shapeRender(glm::vec4(0,0,0,0), glm::vec4(0,1,0,0));
//    else if(frameNum == 70 || frameNum == 87 || frameNum == 106 || frameNum == 123)
//        motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].shapeRender(glm::vec4(0,0,0,0), glm::vec4(0.1,0.1,1,0), 0.3);
//    else
//        motions[TESTNUM].bodies[frameNum % motions[TESTNUM].bodies.size()].shapeRender(glm::vec4(0,0,0,0), glm::vec4(0,0,0,0));
    
//    testMotion1.bodies[frameNum % testMotion1.bodies.size()].shapeRender(glm::vec3(-10,0,0), glm::vec4(1,0,0,0));
//    testMotion2.bodies[frameNum % testMotion2.bodies.size()].shapeRender(glm::vec3(10,0,0), glm::vec4(0,0,1,0));
//
//    for(int i = 0; i < testMotion1.bodies.size()-1; i++){
//        testMotion1.bodies[i].shapeRender(glm::vec3(0,0,0), glm::vec4(1,float(i)/testMotion1.bodies.size(),0 ,0), 0.02);
//        float v = length(testMotion1.bodies[i+1].links[33].getPos() - testMotion1.bodies[i].links[33].getPos());
//        drawCylinder( testMotion1.bodies[i].links[33].getPos(), testMotion1.bodies[i+1].links[33].getPos(), 0.1 , glm::vec4(0, v*v, 1 - v*v,0));

//        testMotion2.bodies[i].shapeRender(glm::vec3(0,0,0), glm::vec4(1,float(i)/testMotion1.bodies.size(),0 ,0), 0.02);
//        float v = length(testMotion2.bodies[i+1].links[33].getPos() - testMotion2.bodies[i].links[33].getPos());
//        drawCylinder( testMotion2.bodies[i].links[33].getPos(), testMotion2.bodies[i+1].links[33].getPos(), 0.1 , glm::vec4(0, v*v, 1 - v*v,0));
//    }
//    int j = 0;
//    for(int i = 0; i < 45; i++){
//        j++;
//        drawCylinder( testMotion1.bodies[i].links[9].getPos() + glm::vec3(0,0,-30+j*0.5), testMotion1.bodies[i+1].links[9].getPos() + glm::vec3(0,0,-30+(j+1)*0.5), 0.05 , glm::vec4(0, 0, 0.8, 0));
//        drawCylinder( testMotion1.bodies[i].links[33].getPos() + glm::vec3(0,0,-30+j*0.5), testMotion1.bodies[i+1].links[33].getPos() + glm::vec3(0,0,-30+(j+1)*0.5), 0.05 , glm::vec4(0, 0, 0.8, 0));
//        if(testMotion1.bodies[i].isBeat){
//            testMotion1.bodies[i].shapeRender(glm::vec3(0,0,-30+j*0.5), glm::vec4(1,0,0,0), 0.1);
//            std::cout  << i << std::endl;
//        }
//        else
//            testMotion1.bodies[i].shapeRender(glm::vec3(0,0,-30+j*0.5), glm::vec4(0, 0.8, 0 ,0), 0.02);
//    }
//    for(int i = 45; i < 59; i++){
//        j++;
//        drawCylinder( testMotion1.bodies[i].links[9].getPos() + glm::vec3(0,0,-30+j*0.5), testMotion1.bodies[i+1].links[9].getPos() + glm::vec3(0,0,-30+(j+1)*0.5), 0.05 , glm::vec4(0, 0, 0.8, 0));
//        drawCylinder( testMotion1.bodies[i].links[33].getPos() + glm::vec3(0,0,-30+j*0.5), testMotion1.bodies[i+1].links[33].getPos() + glm::vec3(0,0,-30+(j+1)*0.5), 0.05 , glm::vec4(0, 0, 0.8, 0));
    
//        testMotion1.bodies[i].shapeRender(glm::vec3(0,0,-30+j*0.5), glm::vec4(0.9, 0.2, 0.4 ,0), 0.02);
//    }
//    for(int i = 59; i < 104; i++){
//        j++;
//        if(i < 103){
//            drawCylinder( testMotion1.bodies[i].links[8].getPos() + glm::vec3(0,0,-30+j*0.5), testMotion1.bodies[i+1].links[8].getPos() + glm::vec3(0,0,-30+(j+1)*0.5), 0.1 , glm::vec4(0, 0.8, 0, 0));
//            drawCylinder( testMotion1.bodies[i].links[33].getPos() + glm::vec3(0,0,-30+j*0.5), testMotion1.bodies[i+1].links[33].getPos() + glm::vec3(0,0,-30+(j+1)*0.5), 0.05 , glm::vec4(0, 0, 0.8, 0));
//        }
//        if(testMotion1.bodies[i].isBeat){
//            testMotion1.bodies[i].shapeRender(glm::vec3(0,0,-30+j*0.5), glm::vec4(1,0,0,0), 0.1);
////            std::cout  << i << std::endl;
//        }
//        else
//            testMotion1.bodies[i].shapeRender(glm::vec3(0,0,-30+j*0.5), glm::vec4(0.1, 0.2, 0.5 ,0), 0.02);
//    }
    drawQuad(glm::vec3(0,0,0), glm::vec3(0,1,0), glm::vec2(2000,2000), glm::vec4(0.8,0.8,0.8,0));
    
//    if(isTransition){
//        trans.bodies[frameNum].shapeRender(glm::vec3(0), glm::vec4(1,0,0,0));
//    }
//    else{
//        beforeM.bodies[frameNum + WINDOW_SIZE_1].shapeRender(glm::vec3(0), glm::vec4(1,0,0,0));
//    }
//    drawQuad(glm::vec3(0,0,0), glm::vec3(0,1,0), glm::vec2(2000,2000));
    
//    int n = TESTNUM;
//    if(motions[n].bodies[frameNum % motions[n].bodies.size()].isStart)
//        motions[n].bodies[frameNum % motions[n].bodies.size()].shapeRender(glm::vec3(0), glm::vec4(1,1,0,0));
//    else if(motions[n].bodies[frameNum % motions[n].bodies.size()].isEnd)
//        motions[n].bodies[frameNum % motions[n].bodies.size()].shapeRender(glm::vec3(0), glm::vec4(1,0,1,0));
//    if(motions[n].bodies[frameNum % motions[n].bodies.size()].isMeter)
//        motions[n].bodies[frameNum % motions[n].bodies.size()].shapeRender(glm::vec3(0), glm::vec4(1,1,1,0));
//    else if(motions[n].bodies[frameNum % motions[n].bodies.size()].isBeat)
//        motions[n].bodies[frameNum % motions[n].bodies.size()].shapeRender(glm::vec3(0), glm::vec4(0,1,1,0));
//    else
//        motions[n].bodies[frameNum % motions[n].bodies.size()].shapeRender();
//    drawQuad(glm::vec3(0,0,0), glm::vec3(0,1,0), glm::vec2(2000,2000));
}

void initFrame() {
    frameNum = 1;
}
void controlFrame(char c) {
    if(c == '[') frameNum--;
    else if(c == ']') frameNum++;
    
    if(frameNum < 0) frameNum = 0;
    std::cout << frameNum << std::endl;
}


int main(int argc, const char * argv[]) {
    
    srand(time(NULL));
    init(path, bvh, motions, 30);
    
//     make Point Clouds
    PC::copyMotion(motions, copyMotions);
    PC::alignRoot(copyMotions);
    PC::createPoint(copyMotions, 1);
    PC::createPC(copyMotions, WINDOW_SIZE_1);
    PC::readyPoseCheck(copyMotions, motions);

    // Meter Check
//    BC::beatCheck(motions[TESTNUM], WINDOW_SIZE_1);
    int c = 0;
    for(auto &m : motions){
//        std::cout << c << std::endl;
        if(c != 2 && c != 3 && c != 12 && c != 23 && c != 22 && c != 36)
            BC::beatCheck(m, WINDOW_SIZE_1);
        c++;
    }


    // make fetureMap
//    for(int i = 0; i < copyMotions.size(); i++) for(int j = 0; j <= i; j++) {
//        std::cout << i << " " << j << std::endl;
//        Eigen::MatrixXf distanceMap;
//        PC::getDistance(copyMotions[i], copyMotions[j], distanceMap, i, j);
////        PC::getLocalMinima(distanceMap, 0.3, WINDOW_SIZE_1, i, j);
//    }
//    for(auto &m : copyMotions) m.clear();
//
//    // make graph
    for(int i = 0; i < motions.size(); i++) for(int j = 0; j <= i; j++) {
        constructEdge(i, j, graph, motions);
    }
    graph.sortEdgeCand();
    constructNode(graph, motions);
    graph.setNodeMap();
//    graph.setPruning();
    graph.printNodeMap();
//    graph.printEdges();
//    graph.printNodes();
//    graph.pruneCheck();

//    // random init
//    before = graph.nodeMap.find(graph.findEdge(s.first,s.second))->second[0];
//    for(int i = before->startFrame-WINDOW_SIZE_1; i < before->endFrame+WINDOW_SIZE_1; i++) {
//        beforeM.add(motions[before->motionNum].bodies[i]);
//    }
    
//    beforeM.frameNum = before->endFrame - before->startFrame + 2*WINDOW_SIZE_1;
//    beforeM = motionWarping(beforeM, 60);
    
//    for(auto & n : graph.nodeList){
//        for(int i = n->startFrame-WINDOW_SIZE_1/2; i < n->endFrame+WINDOW_SIZE_1/2; i++){
//            testMotion.add(motions[n->motionNum].bodies[i]);
//        }
//    }
    
//    for(auto &n : graph.nodeList) {
//        for(int i = n->startFrame; i < n->endFrame; i++) {
//            testMotion.add(motions[n->motionNum].bodies[i]);
//        }
//
//        testMotion1 = motionWarping(testMotion, 60);
//        std::cout << testMotion1.bodies.size() << std::endl;
//        for(auto & b : testMotion1.bodies) testMotion2.add(b);
//        testMotion.clear();
//        testMotion1.clear();
//    }
     
    
    
    testMotion = syncMotions(graph, motions, 120, 30, 15);
    
    
    
    
    // 논문 결과용
//    Motion m1;
//    Motion m2;
//    Motion m3;
//    Motion t1;
//    Motion t2;
//
//    Motion mm1;
//    Motion mm2;
//    Motion mm3;
//    Motion tt1;
//    Motion tt2;
//
//
//    Node * n1 = graph.nodeMap.find(graph.findEdge(31,131))->second;
//    Node * n2 = graph.nodeMap.find(graph.findEdge(27,102))->second;
    
//    Node * n1 = graph.nodeMap.find(graph.findEdge(25,204))->second;
//    Node * n2 = graph.nodeMap.find(graph.findEdge(22,168))->second;
    
//    Node * n1 = graph.nodeMap.find(graph.findEdge(6,41))->second;
//    Node * n2 = graph.nodeMap.find(graph.findEdge(22,758))->second;
    
//    Node * n3 = graph.nodeMap.find(graph.findEdge(35,398))->second;
    
//    m1  = setMotion(motions, n1, 45, 0);
//    m2  = setMotion(motions, n2, 60, 1);
//    mm1 = setMotion(motions, n1, 60, 3);
//    mm2 = setMotion(motions, n2, 60, 1);
//
//    motionBlending(m1, m2, 15);
//    motionBlending(mm1, mm2, 15);
//
//    t1 = makeTransition(m1, m2, 15);
//    tt1 = makeTransition1(mm1, mm2, 15);
//
//    for(int i = 0; i < m1.frameNum; i++) {
//        testMotion1.add(m1.bodies[i]);
//    }
//    for(int i = 0; i < t1.frameNum; i++) {
//        testMotion1.add(t1.bodies[i]);
//    }
//    for(int i = 15; i < m2.frameNum; i++) {
//        testMotion1.add(m2.bodies[i]);
//    }
//
//    for(int i = 0; i < mm1.frameNum-15; i++) {
//        testMotion2.add(mm1.bodies[i]);
//    }
//    for(int i = 0; i < tt1.frameNum; i++) {
//        testMotion2.add(tt1.bodies[i]);
//    }
//    for(int i = 15; i < mm2.frameNum; i++) {
//        testMotion2.add(mm2.bodies[i]);
//    }
    
//    for(int i = 0; i < m1.frameNum; i++) testMotion1.add(m1.bodies[i]);
//    for(int i = 0; i < t1.frameNum; i++) testMotion1.add(t1.bodies[i]);
//    for(int i = 8; i < m2.frameNum; i++) testMotion1.add(m2.bodies[i]);
//    for(int i = 0; i < t2.frameNum; i++) testMotion1.add(t2.bodies[i]);
//    for(int i = 8; i < m3.frameNum; i++) testMotion1.add(m3.bodies[i]);
//
//    for(int i = 0; i < mm1.frameNum-8; i++) testMotion2.add(mm1.bodies[i]);
//    for(int i = 0; i < tt1.frameNum; i++)   testMotion2.add(tt1.bodies[i]);
//    for(int i = 8; i < mm2.frameNum-8; i++) testMotion2.add(mm2.bodies[i]);
//    for(int i = 0; i < tt2.frameNum; i++)   testMotion2.add(tt2.bodies[i]);
//    for(int i = 8; i < mm3.frameNum; i++)   testMotion2.add(mm3.bodies[i]);
    
//    motionBlending(m1, m2, 15);
//    motionBlending(mm1, mm2, 15);
    
//    t1 = makeTransition(m1, m2, 15);
//    tt1 = makeTransition1(mm1, mm2, 15);
    
//    for(int i = 0; i < m1.frameNum; i++) {
//        m1.bodies[i].test = 0;
//        testMotion1.add(m1.bodies[i]);
//    }
//    for(int i = 0; i < t1.frameNum; i++) {
//        t1.bodies[i].test = 1;
//        testMotion1.add(t1.bodies[i]);
//    }
//    for(int i = 15; i < m2.frameNum; i++) {
//        m1.bodies[i].test = 2;
//        testMotion1.add(m2.bodies[i]);
//    }
//    for(int i = 0; i < t2.frameNum; i++) testMotion1.add(t2.bodies[i]);
//    for(int i = 15; i < m3.frameNum; i++) testMotion1.add(m3.bodies[i]);
    
//    for(int i = 40; i < mm1.frameNum-15; i++) testMotion2.add(mm1.bodies[i]);
//    for(int i = 0; i < tt1.frameNum; i++)   testMotion2.add(tt1.bodies[i]);
//    for(int i = 15; i < mm2.frameNum-40; i++) testMotion2.add(mm2.bodies[i]);
//    for(int i = 0; i < tt2.frameNum; i++)   testMotion2.add(tt2.bodies[i]);
//    for(int i = 15; i < mm3.frameNum; i++)   testMotion2.add(mm3.bodies[i]);
    
//    std::cout << testMotion1.bodies.size() << " " << testMotion2.bodies.size() << std::endl;
//     render1
    JGL::Window* window = new JGL::Window(1920, 1024, "simulation");
    window->alignment(JGL::ALIGN_ALL);
    animView = new AnimView(0, 0, 1920, 1024);
    animView->initFunction = initFrame;
    animView->controlFunction = controlFrame;
    animView->renderFunction = render;
    animView->frameFunction = frame;
    window->show();
    JGL::_JGL::run();
    
    return 0;
}
//Node * n1 = graph.nodeMap.find(graph.findEdge(31,131))->second;
//Node * n2 = graph.nodeMap.find(graph.findEdge(27,102))->second;
