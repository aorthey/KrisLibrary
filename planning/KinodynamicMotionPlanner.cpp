#include "KinodynamicMotionPlanner.h"
#include <graph/Callback.h>
#include <algorithm>
#include <errors.h>
#include <utils/EZTrace.h>
#include <math/random.h>
using namespace std;

typedef KinodynamicTree::Node Node;

// ClosestMilestoneCallback: finds the closest milestone to x 
struct ClosestMilestoneCallback : public Node::Callback
{
  ClosestMilestoneCallback(CSpace* s,const Config& _x)
    :space(s),closestDistance(Inf),x(_x),closestMilestone(NULL)
  {}
  virtual void Visit(Node* n) {
    Real d = space->Distance(x,*n);
    if(d < closestDistance) {
      closestDistance = d;
      closestMilestone = n;
    }
  }
  CSpace* space;
  Real closestDistance;
  const Config& x;
  Node* closestMilestone;
};

struct PickCallback : public Node::Callback
{
  PickCallback(int _k) :i(0),k(_k),res(NULL) {}
  virtual void Visit(Node* n) { 
    if(i==k) res=n;
    i++;
  }
  virtual bool Stop() { return (res!=NULL); }

  int i;
  int k;
  Node* res;
};

struct VectorizeCallback : public Node::Callback
{
  virtual void Visit(Node* n) { 
    nodes.push_back(n);
  }

  vector<Node*> nodes;
};










KinodynamicTree::KinodynamicTree(KinodynamicCSpace* s)
  :space(s),root(NULL),Nreach(0)
{
}

KinodynamicTree::~KinodynamicTree()
{
  SafeDelete(root);
}

void KinodynamicTree::EnableReachableSet(uint Nreach_)
{
  Nreach = Nreach_;
}

void KinodynamicTree::Init(const State& initialState)
{
  Clear();
  root = new Node(initialState);
  AddReachableSet(root);
  index.push_back(root);
}

void KinodynamicTree::Clear()
{
  index.clear();
  SafeDelete(root);
}

void KinodynamicTree::AddReachableSet(Node* milestone)
{
  if(Nreach<=0) return;

  milestone->edgeFromParent().reach.clear();
  //uint Nreach = milestone->edgeFromParent().Nreach;
  for(uint i = 0; i < Nreach; i++){
    State xreach;
    ControlInput ureach;
    space->SampleControl(*milestone,ureach);
    space->SimulateEndpoint(*milestone,ureach,xreach);
    milestone->edgeFromParent().reach.push_back(xreach);
    milestone->edgeFromParent().ureach.push_back(ureach);
  }
}

Node* KinodynamicTree::AddMilestone(Node* parent, const ControlInput& u,const State& x)
{
  FatalError("AddMilestone(parent,u,x) deprecated");

  Node* c=parent->addChild(x);
  c->edgeFromParent().u = u;
  c->edgeFromParent().e = NULL;

  AddReachableSet(c);

  index.push_back(c);
  return c;
}

Node* KinodynamicTree::AddMilestone(Node* parent,const ControlInput& u,const vector<State>& path,const SmartPointer<EdgePlanner>& e)
{
  Node* c;
  if(e->Start() == *parent) {
    c=parent->addChild(e->Goal());
  }
  else {
    Assert(e->Goal() == *parent);
    c=parent->addChild(e->Start());
  }
  c->edgeFromParent().u = u;
  c->edgeFromParent().path = path;
  c->edgeFromParent().e = e;
  AddReachableSet(c);
  index.push_back(c);
  return c;
}

void KinodynamicTree::AddPath(Node* n0,const KinodynamicMilestonePath& path,std::vector<Node*>& res)
{
  Assert(*n0 == path.milestones[0]);
  res.resize(0);
  for(size_t i=0;i<path.edges.size();i++) {
    res.push_back(AddMilestone(n0,path.controls[i],path.paths[i],path.edges[i]));
    n0=res.back();
  }
}

void KinodynamicTree::Reroot(Node* n)
{
  FatalError("TODO: KinodynamicTree::Reroot");
}

double KinodynamicTree::ComputeDistance(Node* nlhs, const State& xrhs) const
{
    double d=space->Distance(*nlhs, xrhs);

    uint Nreach = nlhs->edgeFromParent().reach.size();
    if(Nreach>0){
      double localclosest = Inf;
      for(uint j = 0; j < Nreach; j++){
        double dreach = space->Distance(nlhs->edgeFromParent().reach.at(j),xrhs);
        if(dreach<localclosest){
          localclosest = dreach;
        }
      }
      if(localclosest>=d){
        //no progress towards x has been made by any member of reachable set of 
        d = Inf;
      }
    }
    return d;
}
Node* KinodynamicTree::FindClosest(const State& x) const
{
  EZCallTrace tr("KinodynamicTree::FindClosest()");
  
  if(!root) return NULL;

  //iterating is faster
  //Node* closest=index[0];
  Node* closest=NULL;
  double dclosest=Inf;
  for(size_t i=0;i<index.size();i++) {

    double d = ComputeDistance(index.at(i), x);
    if(d < dclosest) {
      dclosest=d;
      closest=index[i];
    }
  }
  return closest;
    /*
  ClosestMilestoneCallback callback(space,x);
  root->DFS(callback);
  return callback.closestMilestone;
    */
}

Node* KinodynamicTree::PickRandom() const
{
  EZCallTrace tr("KinodynamicTree::PickRandom()");

  if(!root) return NULL;

  return index[RandInt(index.size())];

  /*
  CountCallback<Node*> count;
  root->DFS(count);
  PickCallback pick(RandInt(count.count));
  root->DFS(pick);
  Assert(pick.res != NULL);
  return pick.res;
  */
}

Node* KinodynamicTree::ApproximateRandomClosest(const State& x,int numIters) const
{
  EZCallTrace tr("KinodynamicTree::ApproximateRandomClosest()");

  if(!root) return NULL;

  Assert(numIters > 0);
  Real dclosest=Inf;
  Node* closest=NULL;
  for(int i=0;i<numIters;i++) {
    Node* n=index[RandInt(index.size())];
    Real d=space->Distance(x,*n);
    if(d < dclosest) {
      dclosest=d;
      closest=n;
    }
  }
  return closest;
  /*
  VectorizeCallback callback;
  root->DFS(callback);
  Assert(numIters > 0);
  Real dclosest=Inf;
  Node* closest=NULL;
  for(int i=0;i<numIters;i++) {
    Node* n=callback.nodes[RandInt(callback.nodes.size())];
    Real d=space->Distance(x,*n);
    if(d < dclosest) {
      dclosest=d;
      closest=n;
    }
  }
  return closest;
  */
}

void KinodynamicTree::GetPath(Node* start,Node* goal,KinodynamicMilestonePath& path)
{
  path.Clear();
  Node* n=goal;
  while(n != start) {
    if(n == NULL) FatalError("KinodynamicTree::GetPath: nodes specified on tree are not valid!");
    path.milestones.push_back(*n);
    path.controls.push_back(n->edgeFromParent().u);
    path.paths.push_back(n->edgeFromParent().path);
    path.edges.push_back(n->edgeFromParent().e);

    n = n->getParent();
  }
  path.milestones.push_back(*n);

  //flip the vectors
  reverse(path.milestones.begin(),path.milestones.end());
  reverse(path.controls.begin(),path.controls.end());
  reverse(path.paths.begin(),path.paths.end());
  reverse(path.edges.begin(),path.edges.end());
}

void KinodynamicTree::DeleteSubTree(Node* n)
{
  EZCallTrace tr("KinodynamicTree::DeleteSubTree()");
  if(n == root) root = NULL;
  Node* p=n->getParent();
  if(p) p->detachChild(n);
  delete n;  //this automatically deletes n and all children

  VectorizeCallback callback;
  if(root) root->DFS(callback);
  index = callback.nodes;
}





RRTKinodynamicPlanner::RRTKinodynamicPlanner(KinodynamicCSpace* s)
  :space(s),goalSeekProbability(0.1),goalSet(NULL),tree(s),goalNode(NULL)
{}

Node* RRTKinodynamicPlanner::Plan(int maxIters)
{
  if(!goalSet) {
    fprintf(stderr,"RRTKinodynamicPlanner::Plan(): Warning, goalSet is NULL!\n");
    fprintf(stderr,"   Press enter to continue\n");
    getchar();
  }
  if(goalSet && goalSet->IsFeasible(*tree.root)) {
    goalNode = tree.root;
    return tree.root;
  }
  std::cout << "RRT KinodynamicCSpace Planner Start..." << std::endl;
  for(int i=0;i<maxIters;i++) {
    Node* n=Extend();
    if(n){
      std::cout << "iteration " << i << "/" << maxIters << ":" << *n << '\r';
    }else{
      std::cout << "iteration " << i << "/" << maxIters << '\r';
    }
    //if(i%100==0){
    //  std::cout << "iteration " << i << "/" << maxIters << ":" << '\r';
    //}
    if(n && goalSet && goalSet->IsFeasible(*n)) {
      goalNode = n;
      return n;
    }
  }
  return NULL;
}

void RRTKinodynamicPlanner::Init(const State& xinit)
{
  goalNode = NULL;
  tree.Init(xinit);
}

void RRTKinodynamicPlanner::PickDestination(State& xdest)
{
  if(goalSet && RandBool(goalSeekProbability)) {
    goalSet->Sample(xdest);
  }
  else {
    space->Sample(xdest);
  }
}

Node* RRTKinodynamicPlanner::Extend()
{
  State xdest;
  PickDestination(xdest);
  return ExtendToward(xdest);
}


Node* RRTKinodynamicPlanner::ExtendToward(const State& xdest)
{
  const bool DEBUG = false;
  //EZCallTrace tr("RRTKinodynamicPlanner::Extend()");
  //std::cout << "[RRT] Extend towards "<< xdest << std::endl;
  Node* n=tree.FindClosest(xdest);
  if(!n) return NULL;
  ControlInput u;
  if(DEBUG) std::cout << std::string(80, '-') << std::endl;
  if(DEBUG) std::cout << "[RRT] Extend towards "<< xdest << std::endl;
  PickControl(*n,xdest,u);
  if(DEBUG) std::cout << "[RRT] Best control "<< u << std::endl;
  Assert(space->IsValidControl(*n,u));
  vector<State> path;
  space->Simulate(*n,u,path);
  if(DEBUG) std::cout << "[RRT] New point "<< path.back() << std::endl;
  if(!space->IsFeasible(path.back())) {
    if(DEBUG) printf("Edge endpoint is not feasible\n");
    return NULL;
  }
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(e->IsVisible()) {
    Node *nn = tree.AddMilestone(n,u,path,e);

    if(DEBUG) std::cout << "[RRT] add milestone input:"<<*n<< std::endl;
    if(DEBUG) std::cout << "[RRT] edgestart:"<< e->Start() << std::endl;
    if(DEBUG) std::cout << "[RRT] edgegoal :"<< e->Goal() << std::endl;
    if(DEBUG) std::cout << "[RRT] add milestone output:"<<*nn<< std::endl;
    return nn;
  }
  else {
    printf("Edge is not visible\n");
    delete e;
    return NULL;
  }
}

bool RRTKinodynamicPlanner::IsDone() const
{
  return goalNode != NULL;
}

void RRTKinodynamicPlanner::CreatePath(KinodynamicMilestonePath& path) const
{
  Assert(goalNode != NULL);
  tree.GetPath(tree.root,goalNode,path);
}

void RRTKinodynamicPlanner::PickControl(const State& x0, const State& xDest, ControlInput& u) {
  space->BiasedSampleControl(x0,xDest,u);
}




LazyRRTKinodynamicPlanner::LazyRRTKinodynamicPlanner(KinodynamicCSpace* s)
  :RRTKinodynamicPlanner(s)
{}

Node* LazyRRTKinodynamicPlanner::Plan(int maxIters)
{
  if(!goalSet) {
    fprintf(stderr,"LazyRRTKinodynamicPlanner::Plan(): Warning, goalSet is NULL!\n");
    fprintf(stderr,"   Press enter to continue\n");
    getchar();
  }
  if(goalSet && goalSet->IsFeasible(*tree.root)) {
    goalNode = tree.root;
    return tree.root;
  }
  for(int i=0;i<maxIters;i++) {
    Node* n=Extend();
    if(n && goalSet && goalSet->IsFeasible(*n)) {
      if(CheckPath(n)) {
	goalNode = n;
	return n;
      }
    }
  }
  return NULL;
}

Node* LazyRRTKinodynamicPlanner::ExtendToward(const State& xdest)
{
  EZCallTrace tr("LazyRRTKinodynamicPlanner::Extend()");
  //Node* n=tree.FindClosest(xdest);
  Node* n=tree.ApproximateRandomClosest(xdest,10);
  ControlInput u;
  PickControl(*n,xdest,u);
  Assert(space->IsValidControl(*n,u));
  vector<State> path;
  space->Simulate(*n,u,path);
  if(space->IsFeasible(path.back())) {
    EdgePlanner* e=space->TrajectoryChecker(path);
    return tree.AddMilestone(n,u,path,e);
  }
  return NULL;
}

struct NodeWithPriority
{
  Node* n;

  NodeWithPriority(Node* _n) :n(_n) {}
  operator Node* () const { return n; }
  Node* operator -> () const { return n; }
  inline bool operator < (const NodeWithPriority& b) const {
    return n->edgeFromParent().e->Priority() < b->edgeFromParent().e->Priority();
  }
};

bool LazyRRTKinodynamicPlanner::CheckPath(Node* n)
{
  EZCallTrace tr("LazyRRTKinodynamicPlanner::CheckPath()");
  //add all nodes up to n, except root
  priority_queue<NodeWithPriority,vector<NodeWithPriority> > q;
  while(n != tree.root) {
    q.push(n);
    n = n->getParent();
    Assert(n != NULL);
  }
  while(!q.empty()) {
    n=q.top(); q.pop();
    EdgePlanner* e=n->edgeFromParent().e;
    if(!e->Done()) {
      if(!e->Plan()) {
	//disconnect n from the rest of the tree
	tree.DeleteSubTree(n);
	return false;
      }
      q.push(n);
    }
  }
  return true;
}






BidirectionalRRTKP::BidirectionalRRTKP(KinodynamicCSpace* s)
  :space(s),start(s),goal(s),connectionTolerance(1.0)
{
  bridge.nStart=NULL;
  bridge.nGoal=NULL;
}

void BidirectionalRRTKP::Init(const State& xinit,const State& xgoal)
{
  start.Init(xinit);
  goal.Init(xgoal);

  bridge.nStart=NULL;
  bridge.nGoal=NULL;
}

bool BidirectionalRRTKP::IsDone() const
{
  return bridge.nStart != NULL;
}

bool BidirectionalRRTKP::Plan(int maxIters)
{
  double bestDist = Math::dInf;
  for(int iters=0;iters<maxIters;iters++) {
    if(RandBool()) {
      Node* s=ExtendStart();
      if(s) {
        Node* g=goal.FindClosest(*s);

        double d = space->Distance(*s,*g);
        if(d<bestDist){
          bestDist = d;
          std::cout << d << "/" << connectionTolerance << " to " << *s << std::endl;
        }

        if(d < connectionTolerance && ConnectTrees(s,g)){
          return true;
        }
      }
    }
    else {
      Node* g=ExtendGoal();
      if(g) {
        Node* s=start.FindClosest(*g);
        if(space->Distance(*s,*g) < connectionTolerance && ConnectTrees(s,g)) return true;
      }
    }
  }
  return false;
}

Node* BidirectionalRRTKP::ExtendStart()
{
  EZCallTrace tr("BidirectionalRRTKP::ExtendStart()");
  State xdest;
  space->Sample(xdest);
  Node* n=start.FindClosest(xdest);
  ControlInput u;
  PickControl(*n,xdest,u);
  Assert(space->IsValidControl(*n,u));
  vector<State> path;
  space->Simulate(*n,u,path);
  if(!space->IsFeasible(path.back()))
    return NULL;
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(e->IsVisible())
    return start.AddMilestone(n,u,path,e);
  else {
    delete e;
    return NULL;
  }
}

Node* BidirectionalRRTKP::ExtendGoal()
{
  EZCallTrace tr("BidirectionalRRTKP::ExtendGoal()");
  State xdest;
  space->Sample(xdest);
  Node* n=goal.FindClosest(xdest);
  ControlInput u;
  PickReverseControl(*n,xdest,u);
  Assert(space->IsValidReverseControl(*n,u));
  vector<State> path;
  space->ReverseSimulate(*n,u,path);
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(e->IsVisible())
    return goal.AddMilestone(n,u,path,e);
  else delete e;
  return NULL;
}

bool BidirectionalRRTKP::ConnectTrees(Node* a,Node* b)
{
  EZCallTrace tr("BidirectionalRRTKP::ConnectTrees()");
  if(space->ConnectionControl(*a,*b,bridge.u)) {
    if(space->IsValidControl(*a,bridge.u)) {
      space->Simulate(*a,bridge.u,bridge.path);
      Real d=space->Distance(bridge.path.back(),*b);
      if(d >= 1e-3) {
	fprintf(stderr,"BidirectionRRTKP: error detected in CSpace's ConnectionControl() method, distance %g\n",d);;
      }
      Assert(d < 1e-3);
      bridge.e = space->TrajectoryChecker(bridge.path);
      if(bridge.e->IsVisible()) {
	bridge.nStart=a;
	bridge.nGoal=b;
	return true;
      }
      else return false;
    }
    else return false;
  }
  else return false;
}

void BidirectionalRRTKP::PickControl(const State& x0, const State& xDest, ControlInput& u) {
  space->BiasedSampleControl(x0,xDest,u);
}

void BidirectionalRRTKP::PickReverseControl(const State& x1, const State& xStart, ControlInput& u) {
  space->BiasedSampleReverseControl(x1,xStart,u);
}


void BidirectionalRRTKP::CreatePath(KinodynamicMilestonePath& path)
{
  Assert(bridge.nStart != NULL && bridge.nGoal != NULL);  
  KinodynamicMilestonePath pStart,pGoal;
  start.GetPath(start.root,bridge.nStart,pStart);
  goal.GetPath(goal.root,bridge.nGoal,pGoal);

  path.milestones.resize(pStart.milestones.size()+pGoal.milestones.size());
  copy(pStart.milestones.begin(),pStart.milestones.end(),path.milestones.begin());
  reverse_copy(pGoal.milestones.begin(),pGoal.milestones.end(),path.milestones.begin()+pStart.milestones.size());

  path.controls.resize(pStart.controls.size()+pGoal.controls.size()+1);
  copy(pStart.controls.begin(),pStart.controls.end(),path.controls.begin());
  path.controls[pStart.controls.size()] = bridge.u;
  reverse_copy(pGoal.controls.begin(),pGoal.controls.end(),path.controls.begin()+pStart.controls.size()+1);

  path.edges.resize(pStart.edges.size()+pGoal.edges.size()+1);
  copy(pStart.edges.begin(),pStart.edges.end(),path.edges.begin());
  path.edges[pStart.edges.size()] = bridge.e;
  reverse_copy(pGoal.edges.begin(),pGoal.edges.end(),path.edges.begin()+pStart.edges.size()+1);

  path.paths.resize(pStart.paths.size()+pGoal.paths.size()+1);
  copy(pStart.paths.begin(),pStart.paths.end(),path.paths.begin());
  path.paths[pStart.paths.size()] = bridge.path;
  reverse_copy(pGoal.paths.begin(),pGoal.paths.end(),path.paths.begin()+pStart.paths.size()+1);
}

UnidirectionalRRTKP::UnidirectionalRRTKP(KinodynamicCSpace* s)
  :BidirectionalRRTKP(s) { }

void UnidirectionalRRTKP::CreatePath(KinodynamicMilestonePath& path) const
{
  Assert(goalNode != NULL);
  start.GetPath(start.root,goalNode,path);
}
bool UnidirectionalRRTKP::Plan(int maxIters)
{
  double bestDist = Math::dInf;
  for(int iters=0;iters<maxIters;iters++) {
    Node* n=ExtendStart();
    if(n) {
      Node *g = goal.root;
      double d = space->Distance(*n,*g);
      if(d<bestDist){
        bestDist = d;
        std::cout << d << "/" << connectionTolerance << " to " << *n << std::endl;
      }
    }
  }
  return false;
}

RGRRT::RGRRT(KinodynamicCSpace* s, uint Nreach)
  :space(s),goalSeekProbability(0.1),tree(s),goalNode(NULL)
{
  tree.EnableReachableSet(Nreach);
}


Node* RGRRT::Plan(int maxIters)
{
  if(!goalSet) {
    fprintf(stderr,"RRTKinodynamicPlanner::Plan(): Warning, goalSet is NULL!\n");
    fprintf(stderr,"   Press enter to continue\n");
    getchar();
  }
  if(goalSet && goalSet->IsFeasible(*tree.root)) {
    goalNode = tree.root;
    return tree.root;
  }
  for(int i=0;i<maxIters;i++) {
    Node* n=Extend();
    //std::cout << "iteration " << i << "/" << maxIters << ":" << *n << std::endl;
    if(n && goalSet && goalSet->IsFeasible(*n)) {
      goalNode = n;
      return n;
    }
  }
  return NULL;
}

void RGRRT::Init(const State& xinit)
{
  goalNode = NULL;
  tree.Init(xinit);
}

void RGRRT::PickDestination(State& xdest)
{
  if(goalSet && RandBool(goalSeekProbability)) {
    goalSet->Sample(xdest);
  }
  else {
    space->Sample(xdest);
  }
}

Node* RGRRT::Extend()
{
  State xdest;
  PickDestination(xdest);
  return ExtendToward(xdest);
}



Node* RGRRT::ExtendToward(const State& xdest)
{
  const bool DEBUG = false;
  Node* n=tree.FindClosest(xdest);
  if(!n) return NULL;

  ControlInput u;
  PickControl(n,xdest,u);
  Assert(space->IsValidControl(*n,u));
  vector<State> path;
  space->Simulate(*n,u,path);

  if(!space->IsFeasible(path.back())) {
    printf("Edge endpoint is not feasible\n");
    return NULL;
  }
  EdgePlanner* e=space->TrajectoryChecker(path);
  if(e->IsVisible()) {
    Node *nn = tree.AddMilestone(n,u,path,e);

    if(DEBUG) std::cout << "[RRT] add milestone input:"<<*n<< std::endl;
    if(DEBUG) std::cout << "[RRT] edgestart:"<< e->Start() << std::endl;
    if(DEBUG) std::cout << "[RRT] edgegoal :"<< e->Goal() << std::endl;
    if(DEBUG) std::cout << "[RRT] add milestone output:"<<*nn<< std::endl;
    return nn;
  }
  else {
    printf("Edge is not visible\n");
    delete e;
    return NULL;
  }
}

bool RGRRT::IsDone() const
{
  return goalNode != NULL;
}

void RGRRT::CreatePath(KinodynamicMilestonePath& path) const
{
  Assert(goalNode != NULL);
  tree.GetPath(tree.root,goalNode,path);
}


void RGRRT::PickControl(const Node* n, const State& xDest, ControlInput& u) {
  //space->BiasedSampleControl(*n,xDest,u);
  //return;

  uint Nreach = n->edgeFromParent().reach.size();
  if(Nreach>0){
    double d = space->Distance(*n,xDest);

    ControlInput bestu;
    double localclosest = Inf;
    for(uint j = 0; j < Nreach; j++){
      double dreach = space->Distance(n->edgeFromParent().reach.at(j),xDest);
      if(dreach < localclosest){
        localclosest = dreach;
        bestu = n->edgeFromParent().ureach.at(j);
      }
    }
    if(localclosest>=d){
      //no progress towards x has been made by any member of reachable set of 
      d = Inf;
      std::cout << "Warning: trying to extend a node with out of distance reachable set" << std::endl;
    }
    u = bestu;
  }else{
    std::cout << "Warning: using Reachability-guided RRT without reachable set (will behave like RRT)" << std::endl;
    space->BiasedSampleControl(*n,xDest,u);
  }
}
