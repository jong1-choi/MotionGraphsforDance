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
 
#define WINDOW_SIZE_1 7
 
float frameTime = 0;
unsigned int frameNum = 0;
AnimView* animView;
vector< BVH * > bvh;
vector< Motion > motions;
vector< Motion > copyMotions;
MotionGraph graph;
std::string path = "/Users/choijongwon/Downloads/featureData/cleanup_bvh";
Motion testMotion;


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
    }
}

void render() {
    testMotion.bodies[frameNum % testMotion.bodies.size()].shapeRender(glm::vec4(0,0,0,0), glm::vec4(1,0,0,0));

    drawQuad(glm::vec3(0,0,0), glm::vec3(0,1,0), glm::vec2(2000,2000), glm::vec4(0.8,0.8,0.8,0));
    
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
    
//--------------------------- Point Clouds -----------------------------------------------------------
    PC::copyMotion(motions, copyMotions);
    PC::alignRoot(copyMotions);
    PC::createPoint(copyMotions, 1);
    PC::createPC(copyMotions, WINDOW_SIZE_1);
    PC::readyPoseCheck(copyMotions, motions);

    int c = 0;
    for(auto &m : motions){
        if(c != 2 && c != 3 && c != 12 && c != 23 && c != 22 && c != 36) // exception -> edit later
            BC::beatCheck(m, WINDOW_SIZE_1);
    }


//--------------------------- fetureMap --------------------------------------------------------------
//    for(int i = 0; i < copyMotions.size(); i++) for(int j = 0; j <= i; j++) {
//        std::cout << i << " " << j << std::endl;
//        Eigen::MatrixXf distanceMap;
//        PC::getDistance(copyMotions[i], copyMotions[j], distanceMap, i, j);
//    }
//    for(auto &m : copyMotions) m.clear();
//
//--------------------------- graph ------------------------------------------------------------------
    for(int i = 0; i < motions.size(); i++) for(int j = 0; j <= i; j++) {
        constructEdge(i, j, graph, motions);
    }
    graph.sortEdgeCand();
    constructNode(graph, motions);
    graph.setNodeMap();
//    graph.setPruning();
//    graph.printNodeMap();
//    graph.printEdges();
//    graph.printNodes();
//    graph.pruneCheck();

//--------------------------- make motion ------------------------------------------------------------
    testMotion = syncMotions(graph, motions, 120, 30, 15);

//--------------------------- render -----------------------------------------------------------------
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
