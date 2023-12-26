#ifndef graph_hpp
#define graph_hpp

#include <typeinfo>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include "readBVH.hpp"
#include "motion.hpp"
#include "pfm.hpp"

struct Edge;
struct Node {
    unsigned int motionNum;
    unsigned int startFrame;
    unsigned int endFrame;
    Edge* start;
    Edge* end;
    bool isUse = false;
    bool isMeter = false;
    
    Node( unsigned int m, unsigned int s, unsigned int e, Edge* h, Edge* t ) :
    motionNum(m), startFrame(s), endFrame(e), start(h), end(t){}
    void clear() {
        start = nullptr;
        end = nullptr;
    }
};

struct Edge {
    int num;
    pair<int, int> index;
    vector<pair<Edge*, float>> candidates;
    bool isUse = false;
    bool startMeter = false;
    bool endMeter = false;
    bool isMeter = false;
    
    Edge( pair<int,int> i ) : index(i){}
    void printIndex() {
        std::cout << "("<< index.first << "," << index.second << ") ";
    }
    void addEdge(Edge *edge, float w){
        bool isExist = false;
        for(auto &c : candidates){
            if(c.first->index == edge->index) isExist = true;
        }
        if(isExist == false) candidates.push_back( {edge, w} );
    }
    void pruneEdge() {
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](pair<Edge*, float> e){return !e.first->isUse;}),candidates.end());
    }
    void clear() {
        candidates.clear();
        vector<pair<Edge*, float>>().swap(candidates);
    }
    void printCand() {
        int i = 0;
        for(auto & c : candidates) {
            std::cout << i << " : " << "motion "<< c.first->index.first << "\t" << "frame" << c.first->index.second << std::endl;
        }
    }
};
bool cmp(pair<Edge*, float> a, pair<Edge*, float> b) { return a.second < b.second; }
struct MotionGraph {
    map<pair<int,int>, Edge*> edgeMap;
    map<Edge*, Node*> nodeMap;
    vector<Edge*> edgeList;
    vector<Node*> nodeList;
    
    void addEdgeMap(Edge &e) { edgeMap.insert({e.index, &e}); }
    void addEdgeList(Edge &e) { edgeList.push_back(&e); }
    Edge* findEdge(int motion, int frame){ return edgeMap.find({motion, frame})->second; }
    bool isEdgeExist(int motion, int frame){
        if( edgeMap.find({motion, frame}) != edgeMap.end() ) return 1;
        else return 0;
    }
    void sortEdgeCand() {
        for(auto & e : edgeList){
            sort(e->candidates.begin(), e->candidates.end(),
                 [](pair<Edge*, float> a, pair<Edge*, float> b)->bool
            {
                return a.second < b.second;
            });
        }
//        for(auto & e : edgeList){
//            for(auto & c : e->candidates){
//                std::cout << c.second << " ";
//            }
//            std::cout << std::endl;
//        }
    }
    void addNewEdge(int motion, int frame){
        if( !isEdgeExist(motion,frame) ) {
            Edge *newEdge = new Edge({motion,frame});
            addEdgeList( *newEdge );
            addEdgeMap( *newEdge );
            newEdge->num = edgeList.size()-1;
        }
    }
    void printEdges(){
        std::cout << edgeList.size() << std::endl;
        for( auto &a : edgeList ) {
            std::cout << "origin Edge -> "<< a->index.first << " " << a->index.second << " isUse" << a->isUse << std::endl;
            for( auto &b : a->candidates ){
                    std::cout << "candidates -> "<< b.first->index.first << " " << b.first->index.second << "   \tweights -> " << b.second << " isUse -> " << b.first->isUse << std::endl;
            }
            std::cout << std::endl;
        }
    }
    void printNodeMap(){
        for(auto & e : nodeList){
            std::cout << "motion -> " << e->start->index.first << " frame -> " << e->start->index.second << std::endl;
            std::cout << "motion -> " << nodeMap.find(e->start)->second->motionNum << " start -> " << nodeMap.find(e->start)->second->startFrame << " end -> " << nodeMap.find(e->start)->second->endFrame << std::endl;
            std::cout << std::endl;
        }
    }
    void addNodeList(Node &n) { nodeList.push_back(&n); }
    bool isNodeExist(Edge* e){
        if( nodeMap.find(e) != nodeMap.end() ) return 1;
        else return 0;
    }
    void setNodeMap() {
        for(auto &n : nodeList){
            nodeMap.insert(make_pair(n->start,n));
        }
    }

    
    void printNodes(){
        std::cout << nodeList.size() << std::endl;
        for( auto &n : nodeList ) {
            std::cout << "motion number -> "<< n->motionNum << " frame size -> " << n->endFrame - n->startFrame << std::endl;
            std::cout << "start frame -> "<< n->startFrame << " end frame -> " << n->endFrame << std::endl;
            std::cout << "start Edge -> " << n->start->index.first << " " << n->start->index.second << std::endl;
            std::cout << std::endl;
        }
    }
    
    vector<int> d;
    vector<bool> finished;
    stack<Edge*> s;
    vector<vector<Edge*>> SCC;
    int id = -1;
    int maxSCC = 0;
    
    Edge* dfs(Edge* x) {
        d[x->num] = id++;
        s.push(x);
        int parent = d[x->num];
        for(int i = 0; i < x->candidates.size(); i++) {
            Edge* y = x->candidates[i].first;
            if(d[y->num] == -1) parent = std::min(parent, dfs(y)->num);
            else if(!finished[y->num]) parent = std::min(parent, d[y->num]);
        }
        
        if(parent == d[x->num]) {
            vector<Edge*> scc;
            while(1){
                Edge* t = s.top();
                s.pop();
                scc.push_back(t);
                finished[t->num] = true;
                if(t->num == x->num) break;
            }
            SCC.push_back(scc);
        }
        return edgeList[parent];
    }
    
    void setPruning(){
        d.resize(edgeList.size(), -1);
        finished.resize(edgeList.size(), 0);
        for(int i = 0; i < edgeList.size(); i++) {
            if(d[edgeList[i]->num] == -1) dfs(edgeList[i]);
        }
        for(int i = 0; i < SCC.size(); i++) {
            if( SCC[maxSCC].size() < SCC[i].size() ) maxSCC = i;
        }
        for(int i = 0; i < SCC[maxSCC].size(); i++) {
            SCC[maxSCC][i]->isUse = true;
        }
        for(auto & n : nodeList) {
            if(n->start->isUse && n->end->isUse) {
                n->isUse = true;
            }
//            if(!n->isMeter) n->clear();
        }
        for(auto & e : edgeList) {
            e->pruneEdge();
            if(!e->isUse) {
                edgeMap.erase({e->index});
            }
        }
        nodeList.erase(std::remove_if(nodeList.begin(), nodeList.end(), [](Node* n){return !n->isUse;}),nodeList.end());
        edgeList.erase(std::remove_if(edgeList.begin(), edgeList.end(), [](Edge* e){return !e->isUse;}),edgeList.end());
    }
    
    void pruneCheck(){
        std::cout << "SCC의 개수 : " << SCC.size() << std::endl;
        for(int i = 0; i < SCC.size(); i++) {
            std::cout << i << "번째 SCC: " << i;
            for(int j = 0; j < SCC[i].size(); j++) {
                SCC[i][j]->printIndex();
            }
            std::cout << std::endl;
        }
    }
};

void constructEdge(int num1, int num2, MotionGraph &g, const vector<Motion> &motions) {
    int w,h,c;
    float * data;
    std::string path = "/Users/choijongwon/Downloads/featureData/distanceMap1/";
    path.append(to_string(num1));
    path.append("-");
    path.append(to_string(num2));
    path.append(".pfm");
    std::tie(w,h,c,data) = loadPFM(path);
    for(int i = 0; i < w * h; i++) {
//        if(!data[i]) continue;
        if(!motions[num1].bodies[i%w].isMeter || !motions[num2].bodies[i/w].isMeter) continue;
        g.addNewEdge(num1,i%w);
        g.addNewEdge(num2,i/w);
        if(motions[num1].bodies[i%w].isEnd && motions[num2].bodies[i/w].isStart)
            g.findEdge(num1,i%w)->addEdge(g.findEdge(num2,i/w), data[i]);
        if(motions[num1].bodies[i%w].isStart && motions[num2].bodies[i/w].isEnd)
            g.findEdge(num2,i/w)->addEdge(g.findEdge(num1,i%w), data[i]);
    }
}
void constructNode(MotionGraph &g, const vector<Motion> &motions) {
    for(int i = 0; i < motions.size(); i++) {
        for(int j = 0; j < motions[i].frameNum; j++){
            if( !motions[i].bodies[j].isStart ) continue;
            int e = motions[i].bodies[j].endMeter;
            Node *n = new Node(i, j, e, g.findEdge(i,j), g.findEdge(i,e) );
            g.addNodeList(*n);
        }
    }
}
#endif /* graph_hpp */
